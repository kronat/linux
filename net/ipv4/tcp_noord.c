/* TCP Noordwijk
 *
 * see C. Roseti, E. Kristiansen:
 * “TCP Noordwijk: optimize TCP-based transport over DAMA in satellite network”.
 *
 * (c) 2013 Natale Patriciello, University of Modena and Reggio Emilia
 * Dip. Ingegneria Enzo Ferrari
 */

#include <linux/module.h>
#include <net/tcp.h>

#define BURST_0 2
#define LOG2_BURST_0 1
#define DELTA_0 (500000 >> 3)
#define BETA    200000
#define STABILITY_FACTOR 2 /* TODO: Inglese */

#define U32_MAX     ((u32)~0U)

/*#define jiffies_to_msec(jiffies) (jiffies * 1000 / HZ)*/

/* TODO: Document each values kernel style. */

/* TCP Noordwijk Parameters */
struct noord {
	bool take_rtt; 		/* should we consider the rtt arrived? */
	u32 last_rtt;		/* last rtt arrived */
	u32 fp_rtt;		/* rtt of first ack of the burst  */
	u32 fp_timestamp;	/* Timestamp of the first ack of the burst */

	u32 rtt_min;		/* Minimum RTT encountered */
	u32 ack_count;		/* how many ACKs we have received since last burst reset */

	u32 burst_eff;		/* Real length of burst (in case of pkt loss, eff != len) */
	u32 burst_len;		/* Ideal length of burst */

	u32 delta;		/* ACK dispersion. Calculated only when burst_eff = BURST_0 */

	u8 burst_count;		/* Count of processed burst */

	u32 tx_timer;
	u32 tx_timer_prev;

	u32 loss_snapshot;	/* Snapshot of packet losses taken every STAB_FACT times */
};

#ifdef DEBUG
static void print_sk_info(const char *fun, struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	printk("%s rcv_nxt=%u, snd_nxt=%u, snd.una=%u, cwnd=%u, snd.sml=%u\n", fun,
			tp->rcv_nxt, tp->snd_nxt, tp->snd_una, tp->snd_cwnd, tp->snd_sml);
}

static void print_write_queue(const char *fun, struct sock *sk)
{
	struct sk_buff *skb;
	struct tcp_skb_cb *scb;

	while ((skb = tcp_write_queue_head(sk))) {
		if (skb == tcp_send_head(sk)) {
			printk("%s: Got head of skb.\n", fun);
			break;
		}

		scb = TCP_SKB_CB(skb);
		printk("%s seq=%u, ack=%u\n", fun, scb->seq, scb->ack_seq);
	}
}
#endif

/* Initial Parameters (new transmission, or idle time passed (<-- check it),
 * or RTO expired (<-- check it) */
static void tcp_noord_init(struct sock *sk)
{
	struct noord *ca = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);

#ifdef DEBUG
	print_sk_info("INIT:", sk);
	print_write_queue("INIT:", sk);
#endif

	ca->rtt_min = U32_MAX;

	ca->take_rtt = true;
	ca->ack_count = 0;

	ca->burst_eff = BURST_0;
	ca->burst_len = BURST_0;

	ca->burst_count = 0;

	ca->tx_timer = DELTA_0;
	ca->tx_timer_prev = DELTA_0;

	ca->loss_snapshot = 0;

	ca->delta = 0; /* Useless init. We will update delta the first time we reach
			  the end of the burst */

	/* First time, it is useless. Someone will write in
	 * snd_cwnd the value TCP_CWND_INIT. */
	tp->snd_cwnd = ca->burst_eff;
}


/*
 * Available states (from tcp.h):
 *
 * Can we extract some information from these states?
 *
 *  enum tcp_ca_state {
 *    TCP_CA_Open = 0,
 *    TCP_CA_Disorder = 1,
 *    TCP_CA_CWR = 2,
 *    TCP_CA_Recovery = 3,
 *    TCP_CA_Loss = 4
 *  }
 */
static void tcp_noord_state(struct sock *sk, u8 new_state)
{
	if (new_state == TCP_CA_Open) {
		printk("noord_state: CA_OPEN\n");
		tcp_noord_init(sk);
	}
}

/* Received ACK */
static void tcp_noord_acked(struct sock *sk, u32 pkts_acked, s32 rtt)
{
	struct noord *ca = inet_csk_ca(sk);

	/* dup ack, no rtt sample */
	if (rtt < 0)
		return;

	ca->ack_count += pkts_acked;

	if (ca->rtt_min > rtt)
		ca->rtt_min = rtt;

	ca->last_rtt = rtt;

	/* Make sure cwnd size is fixed across the burst */
	tcp_sk(sk)->snd_cwnd = ca->burst_len;

#ifdef DEBUG
	printk("tcp_noord_acked: pkts_acked = %u, rtt = %d, misalvo=%u\n", pkts_acked, rtt, ca->last_rtt);
	print_sk_info("tcp_noord_acked:", sk);
#endif
}

/* Update the real cwnd size */
static inline void flow_ctrl(struct sock *sk)
{
	struct noord *ca = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);

	u32 actual_loss = tp->lost_out;
	u32 th_burst_len = min(ca->burst_len, tp->rcv_wnd);
	u32 pkt_loss = actual_loss - ca->loss_snapshot;

#ifdef DEBUG
	printk("flow_control: actual_loss=%u, th_burst=%u, pkt_loss=%u\n", actual_loss,
		th_burst_len, pkt_loss);
#endif

	if (th_burst_len < pkt_loss)
		ca->burst_eff = 0;
	else
		ca->burst_eff = th_burst_len - pkt_loss;

	ca->loss_snapshot = actual_loss;

	tp->snd_cwnd = ca->burst_eff;
}


/* Evaluate congestion level, updating (if necessary) the burst_eff length */
static inline void rate_ctrl(struct sock *sk)
{
	struct noord *ca = inet_csk_ca(sk);

	u32 rtt_diff = ca->fp_rtt - ca->rtt_min;

#ifdef DEBUG
	printk("RateControl: old_burst_len=%u, rtt_diff=%u\n", ca->burst_len, rtt_diff);
#endif

	/* Difference between first ack rtt with the min rtt received */
	if(rtt_diff <= BETA) {
		// Rate tracking
		ca->burst_len = (ca->burst_len + BURST_0) >> 1;
#ifdef DEBUG
		printk("RateTracking: burst=%u\n", ca->burst_len);
#endif
	} else {
		// Rate Adjustment
		/* if STAB_FACT == 1, we could have division by 0. Take care of it. */
		ca->burst_len = ca->burst_len * (ca->tx_timer_prev << 3) /
				((ca->tx_timer_prev << 3) + rtt_diff);
#ifdef DEBUG
		printk("RateAdj: timer_prev=%u, new_burst=%u\n",
			(ca->tx_timer_prev << 3), ca->burst_len);
#endif
	}

	flow_ctrl(sk);
}

static void update_burst(struct sock *sk)
{
	struct noord *ca = inet_csk_ca(sk);

	ca->burst_count += 1;
#ifdef DEBUG
	printk("update_burst: burst_count=%u\n", ca->burst_count);
#endif

	if (ca->burst_count == STABILITY_FACTOR) {
		rate_ctrl(sk);
		ca->burst_count = 0;
	}

}

static void tcp_noord_cong_avoid(struct sock *sk, u32 ack, u32 in_flight)
{
	struct noord *ca = inet_csk_ca(sk);
#ifdef DEBUG
	struct tcp_sock *tp = tcp_sk(sk);

	printk("cong_avoid: ack=%u, inf=%u\n", ack, in_flight);
	print_sk_info("cong_avoid:", sk);
	print_write_queue("cong_avoid:", sk);
#endif

	/* Is the last RTT received the one which interest us ? */
	if (ca->take_rtt) {
#ifdef DEBUG
		printk("cong_avoid: preso rtt=%u\n", ca->last_rtt);
#endif
		ca->take_rtt = false;

		/* Take this rtt as the first packet of the burst rtt */
		ca->fp_rtt = ca->last_rtt;
	}

#ifdef DEBUG
	printk("cong_avoid: ack_count=%u, cwnd=%u, loss=%u\n", ca->ack_count, tp->snd_cwnd, tp->lost_out);
#endif

	/* TODO: Check if tp->lost_out interfer in some way */
	/* Check if we exhausted burst */
	if (ca->ack_count >= ca->burst_eff) {
		ca->ack_count = 0;
		ca->take_rtt = true;

		ca->tx_timer_prev = ca->tx_timer;
		ca->tx_timer = jiffies_to_usecs(tcp_time_stamp - ca->fp_timestamp);
#ifdef DEBUG
		printk("cong_avoid: BURST_DONE. timer=%u\n", ca->tx_timer);
#endif

		if (ca->burst_eff == BURST_0) {
			// ts of last ack - ts of first ack / BURST_0
			ca->delta = (ca->tx_timer << 3) >> LOG2_BURST_0;
#ifdef DEBUG
			printk("cong_avoid: ca->delta=%u\n", ca->delta);
#endif
		}

		update_burst(sk);
	}
}

/*
 * Available events:
 *
 *  CA_EVENT_TX_START,       first transmit when no packets in flight
 *  CA_EVENT_CWND_RESTART,   congestion window restart
 *  CA_EVENT_COMPLETE_CWR,   end of congestion recovery
 *  CA_EVENT_FRTO,           fast recovery timeout
 *  CA_EVENT_LOSS,           loss timeout
 *  CA_EVENT_FAST_ACK,       in sequence ack
 *  CA_EVENT_SLOW_ACK,       other ack
 *
 *  Can we use these information?
 */
static void tcp_noord_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct noord * ca = inet_csk_ca(sk);
	switch(event) {
		case CA_EVENT_TX_START:
#ifdef DEBUG
			printk("cwnd_event: TX_START\n");
#endif
			/*
			 * If  we  receive  a  TX_START  event, a  packet is
			 * being  transmitted  while  there  are  no packets
			 * in  flight.  Therefore,  if  we  see  a  TX_START
			 * event  while  ca->ack_count == 0,   we  are  sure
			 * we are  transmitting  the  first  packet  of  the
			 * burst.  In  fact,  if  we  were transmitting some
			 * subsequent packet, either:  a) we  would  already
			 * have received  the  ACKs for the previous packets
			 * (and hence  ack_count would be != 0), or b) there
			 * would  be  packets in flight  (and hence we would
			 * not be receiving a TX_START event).
			 */
			if (ca->ack_count == 0)
				ca->fp_timestamp = tcp_time_stamp;
			break;
		case CA_EVENT_CWND_RESTART:
#ifdef DEBUG
			printk("cwnd_event: CWND_RESTART\n");
#endif
			tcp_sk(sk)->snd_cwnd = ca->burst_len;
			break;
		default:
			/* don't care */
			break;
	}
}

/* XXX Wow. In Noordwijk we don't have slow start, so it is basically flawed. We
 * should return 0, am I right? */
static u32 tcp_noord_ssthresh(struct sock *sk)
{
#ifdef DEBUG
	print_sk_info("ssth:", sk);
	print_write_queue("ssth:", sk);
#endif

	/* return 0; */
	return tcp_reno_ssthresh(sk);
}


/*
 * Fields of this structure are (from tcp.h):
 *
 * initialize private data (optional)
 *   void (*init)(struct sock *sk);
 *
 * cleanup private data  (optional)
 *   void (*release)(struct sock *sk);
 *
 * return slow start threshold (required)
 *   u32 (*ssthresh)(struct sock *sk);
 *
 * lower bound for congestion window (optional)
 *   u32 (*min_cwnd)(const struct sock *sk);
 *
 *  do new cwnd calculation (required)
 *   void (*cong_avoid)(struct sock *sk, u32 ack, u32 in_flight);
 *
 * call before changing ca_state (optional)
 *   void (*set_state)(struct sock *sk, u8 new_state);
 *
 * call when cwnd event occurs (optional)
 *   void (*cwnd_event)(struct sock *sk, enum tcp_ca_event ev);
 *
 * new value of cwnd after loss (optional)
 *   u32  (*undo_cwnd)(struct sock *sk);
 *
 * hook for packet ack accounting (optional)
 *   void (*pkts_acked)(struct sock *sk, u32 num_acked, s32 rtt_us);
 *
 * get info for inet_diag (optional)
 *   void (*get_info)(struct sock *sk, u32 ext, struct sk_buff *skb);
 */
static struct tcp_congestion_ops tcp_noord __read_mostly = {
	.init		= tcp_noord_init,
	.ssthresh	= tcp_noord_ssthresh,
	.cong_avoid	= tcp_noord_cong_avoid,
	.set_state	= tcp_noord_state,
	.cwnd_event	= tcp_noord_cwnd_event,
	.pkts_acked	= tcp_noord_acked,
	.owner		= THIS_MODULE,
	.name		= "noord",
};

static int __init tcp_noord_register(void)
{
	BUILD_BUG_ON(sizeof(struct noord) > ICSK_CA_PRIV_SIZE);
	return tcp_register_congestion_control(&tcp_noord);
}

static void __exit tcp_noord_unregister(void)
{
	tcp_unregister_congestion_control(&tcp_noord);
}

module_init(tcp_noord_register);
module_exit(tcp_noord_unregister);

MODULE_AUTHOR("Natale Patriciello");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TCP NOORDWIJK");
