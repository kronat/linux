/*
 * TCP Wave
 *
 * Copyright 2017 Natale Patriciello <natale.patriciello@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Add some description..
 */

#include <net/tcp.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define WAVE_DEBUG 1

#ifdef WAVE_DEBUG
 #define DBG(msg ...) do { printk("WAVE_DEBUG: " msg); } while (0)
#else
static inline void DBG(const char *msg, ...)
{
}
#endif

static uint init_burst __read_mostly = 10;
static uint min_burst __read_mostly = 3;
static uint init_timer_ms __read_mostly = 500;
static uint beta_ms __read_mostly = 150;

module_param(init_burst, uint, 0644);
MODULE_PARM_DESC(init_burst, "initial burst (segments)");
module_param(min_burst, uint, 0644);
MODULE_PARM_DESC(min_burst, "minimum burst (segments)");
module_param(init_timer_ms, uint, 0644);
MODULE_PARM_DESC(init_timer_ms, "initial timer (ms)");
module_param(beta_ms, uint, 0644);
MODULE_PARM_DESC(beta_ms, "beta parameter (ms)");

/* List for saving the size of sent burst over time */
struct wavetcp_burst_hist {
	u32 size;		/* The burst size */
	struct list_head list;	/* Kernel list declaration */
};

/* TCP Wave private struct */
struct wavetcp {
	/* Tell if the driver is initialized (init has been called) */
	u8 initialized;
	/* Tell if, as sender, the driver is started (after TX_START) */
	u8 started;
	/* The current transmission timer (ms) */
	u16 tx_timer;
	/* The current burst size (segments) */
	u32 burst;
	/* Represents a delta from the burst size of segments sent */
	int delta_segments;
	/* The segments acked in the round */
	u32 pkts_acked;

	/* The memory cache for saving the burst sizes */
	struct kmem_cache *cache;
	/* The burst history */
	struct wavetcp_burst_hist history;

	unsigned long first_ack_time;
};

/* Reset all the values except ca->initialized to 0. When needed (e.g., after a
 * TX_START event) the code will set them at their initial value
 */
static void wavetcp_reset(struct sock *sk)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	ca->started = 0;
	ca->tx_timer = 0;
	ca->burst = 0;
}

/* Called to setup Wave for the current socket after it enters the CONNECTED
 * state (i.e., called after the SYN-ACK is received). The slow start should be
 * 0 (see wavetcp_get_ssthresh) and we set the initial cwnd to the initial
 * burst.
 *
 * After the ACK of the SYN-ACK is sent, the TCP will add a bit of delay to
 * permit the queueing of data from the application, otherwise we will end up
 * in a scattered situation (we have one segment -> send it -> no other segment,
 * don't set the timer -> slightly after, another segment come and we loop).
 *
 * At the first expiration, the cwnd will be large enough to push init_burst
 * segments out.
 */
static void wavetcp_init(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);

	DBG("%u [wavetcp_init]\n", tcp_time_stamp);

	/* Setting the initial Cwnd to 0 will not call the TX_START event */
	tp->snd_ssthresh = 0;
	tp->snd_cwnd = init_burst;

	wavetcp_reset(sk);

	/* Used to avoid to take the SYN-ACK measurements, and for nothing else */
	ca->initialized = 1;

	ca->delta_segments = 0;
	ca->first_ack_time = 0;

	/* Init the history of bwnd */
	INIT_LIST_HEAD(&ca->history.list);

	/* Init our cache pool for the bwnd history */
	ca->cache = KMEM_CACHE(wavetcp_burst_hist, 0);
	BUG_ON(ca->cache == 0);
}

static void wavetcp_release(struct sock *sk)
{
	struct wavetcp *ca = inet_csk_ca(sk);
	struct wavetcp_burst_hist *tmp;
	struct list_head *pos, *q;

	DBG("%u [wavetcp_release]\n", tcp_time_stamp);

	list_for_each_safe(pos, q, &ca->history.list){
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);
		list_del(pos);
		kmem_cache_free(ca->cache, tmp);
	}

	/* Thanks for the cache, we don't need it anymore */
	if (ca->cache != 0)
		kmem_cache_destroy(ca->cache);
}

static void wavetcp_history(struct wavetcp *ca)
{
	struct list_head *pos, *q;
	struct wavetcp_burst_hist *tmp;

	list_for_each_safe(pos, q, &ca->history.list) {
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);
		DBG("STATE: burst %u", tmp->size);
	}
}

/* Please explain that we will be forever in congestion avoidance. */
static u32 wavetcp_recalc_ssthresh(struct sock *sk)
{
	DBG("%u [wavetcp_recalc_ssthresh]\n", tcp_time_stamp);
	return 0;
}

/*
   static void wavetcp_cong_avoid(struct sock *sk, u32 ack, u32 acked)
   {
   }
 */

static void wavetcp_cong_control(struct sock *sk, const struct rate_sample *rs)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	DBG("%u [wavetcp_cong_control] prior_delivered %u, delivered %i, interval_us %li, "
	    "rtt_us %li, losses %i, ack_sack %u, prior_in_flight %u, is_app %i,"
	    " is_retrans %i\n", tcp_time_stamp, rs->prior_delivered,
	    rs->delivered, rs->interval_us, rs->rtt_us, rs->losses,
	    rs->acked_sacked, rs->prior_in_flight, rs->is_app_limited,
	    rs->is_retrans);

	if (!ca->initialized)
		return;
}

static void wavetcp_state(struct sock *sk, u8 new_state)
{
	switch (new_state) {
	case TCP_CA_Open:
		DBG("%u [wavetcp_state] set CA_Open\n", tcp_time_stamp);
		break;
	case TCP_CA_Disorder:
		DBG("%u [wavetcp_state] set CA_Disorder\n", tcp_time_stamp);
		break;
	case TCP_CA_Recovery:
		DBG("%u [wavetcp_state] set CA_Recovery\n", tcp_time_stamp);
		break;
	case TCP_CA_Loss:
		DBG("%u [wavetcp_state] set CA_Loss\n", tcp_time_stamp);
		break;
	default:
		DBG("%u [wavetcp_state] set state %u\n",
		    tcp_time_stamp, new_state);
	}

}

static u32 wavetcp_undo_cwnd(struct sock *sk)
{
	DBG("%u [wavetcp_undo_cwnd]\n", tcp_time_stamp);
	return init_burst;
}

/* Add the size of the burst in the history of bursts */
static void wavetcp_insert_burst(struct wavetcp *ca, u32 burst)
{
	struct wavetcp_burst_hist *cur;

	DBG("%u [wavetcp_insert_burst] burst %u\n", tcp_time_stamp, burst);

	/* Take the memory from the pre-allocated pool */
	cur = (struct wavetcp_burst_hist *)kmem_cache_alloc(ca->cache,
							    GFP_KERNEL);
	BUG_ON(!cur);

	cur->size = burst;
	list_add_tail(&cur->list, &ca->history.list);
}

static void wavetcp_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	if(!ca->initialized)
		return;

	switch (event) {
	case CA_EVENT_TX_START:
		/* first transmit when no packets in flight */
		DBG("%u [wavetcp_cwnd_event] TX_START, set burst to %u and timer to %u ms\n",
		    tcp_time_stamp, init_burst, init_timer_ms);

		if (!ca->started)
			wavetcp_insert_burst(ca, init_burst);

		ca->started = 1;
		ca->burst = init_burst;
		ca->delta_segments = init_burst;
		ca->tx_timer = init_timer_ms;

		wavetcp_history(ca);

		break;
	case CA_EVENT_CWND_RESTART:
		/* congestion window restart */
		DBG("%u [wavetcp_cwnd_event] CWND_RESTART\n", tcp_time_stamp);
		break;
	case CA_EVENT_COMPLETE_CWR:
		/* end of congestion recovery */
		DBG("%u [wavetcp_cwnd_event] COMPLETE_CWR\n", tcp_time_stamp);
		break;
	case CA_EVENT_LOSS:
		/* loss timeout */
		DBG("%u [wavetcp_cwnd_event] EVENT_LOSS\n", tcp_time_stamp);
		break;
	case CA_EVENT_ECN_NO_CE:
		/* ECT set, but not CE marked */
		DBG("%u [wavetcp_cwnd_event] ECN_NO_CE\n", tcp_time_stamp);
		break;
	case CA_EVENT_ECN_IS_CE:
		/* received CE marked IP packet */
		DBG("%u [wavetcp_cwnd_event] ECN_IS_CE\n", tcp_time_stamp);
		break;
	case CA_EVENT_DELAYED_ACK:
		/* Delayed ack is sent */
		DBG("%u [wavetcp_cwnd_event] DELAYED_ACK\n", tcp_time_stamp);
		break;
	case CA_EVENT_NON_DELAYED_ACK:
		/* Non-delayed ack is sent */
		DBG("%u [wavetcp_cwnd_event] NON_DELAYED_ACK\n", tcp_time_stamp);
		break;
	default:
		DBG("%u [wavetcp_cwnd_event] default case\n", tcp_time_stamp);
		break;
	}
}

/* Invoked each time we receive an ACK. Obviously, this function also gets
 * called when we receive the SYN-ACK, but we ignore it thanks to the
 * ca->initialized flag.
 *
 * We close the cwnd of the amount of segments acked, because we don't like
 * sending out segments if the timer is not expired. Without doing this, we
 * would end with cwnd - in_flight > 0.
 */
static void wavetcp_acked(struct sock *sk, const struct ack_sample *sample)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);
	struct wavetcp_burst_hist *tmp;
	struct list_head *pos;

	if (!ca->initialized)
		return;

	DBG("%u [wavetcp_acked] pkts_acked %u, rtt_us %u, in_flight %u "
	    ", cwnd %u\n",
	    tcp_time_stamp, sample->pkts_acked, sample->rtt_us,
	    sample->in_flight, tp->snd_cwnd);

	if (tp->snd_cwnd >= sample->pkts_acked) {
		/* The objective here is to avoid to create space for sending segments.
		 * This is a duty for the timer expiration. */
		tp->snd_cwnd -= sample->pkts_acked;
	} else {
		/* We sent some scattered segments, so the burst segments and
		 * the ACK we get is not aligned.
		 */
		ca->delta_segments += sample->pkts_acked - tp->snd_cwnd;
		tp->snd_cwnd = 0;
	}

	DBG("%u [wavetcp_acked] new window %u in_flight %u delta %i\n",
	    tcp_time_stamp, tp->snd_cwnd, tcp_packets_in_flight(tp),
	    ca->delta_segments);

	BUG_ON(tp->snd_cwnd - (u32)(ca->delta_segments) > tcp_packets_in_flight(tp));

	if (ca->first_ack_time == 0)
		ca->first_ack_time = tcp_time_stamp;
	ca->pkts_acked += sample->pkts_acked;

	pos = ca->history.list.next;
	tmp = list_entry(pos, struct wavetcp_burst_hist, list);
	if (ca->pkts_acked >= tmp->size) {
		/* Do the calculations */
		DBG("%u [wavetcp_acked] reached the burst size %u\n",
		    tcp_time_stamp, tmp->size);

		ca->pkts_acked = 0;

		/* Delete the burst from the history */
		list_del(pos);
		kmem_cache_free(ca->cache, tmp);
		wavetcp_history(ca);
	}
}

/* The TCP informs us that the timer is expired (or has never been set). We can
 * infer the latter by the ca->started flag: if it's false, don't increase the
 * cwnd, because it is at its default value (init_burst) and we still have to
 * transmit the first burst.
 */
static void wavetcp_timer_expired(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);
	u32 current_burst = ca->burst;

	BUG_ON(!ca->initialized);

	if (!ca->started)
		return;

	if (ca->delta_segments < 0) {
		/* In the previous round, we sent more than the allowed burst,
		 * so reduce the current burst.
		 */
		BUG_ON(current_burst > ca->delta_segments);
		current_burst += ca->delta_segments; /* please *reduce* */
		ca->delta_segments = current_burst;

		/* Right now, we should send "current_burst" segments out */

		if (tcp_packets_in_flight(tp) > tp->snd_cwnd) {
			/* For some reasons (e.g., tcp loss probe)
			 * we sent something outside the allowed window.
			 * Add the amount of segments into the burst, in order
			 * to effectively send the previous "current_burst"
			 * segments, but without touching delta_segments.
			 */
			u32 diff = tcp_packets_in_flight(tp) - tp->snd_cwnd;
			current_burst += diff;
			DBG("%u [wavetcp_timer_expired] adding %u to balance "
			    "segments sent out of window", tcp_time_stamp,
			    diff);
		}
	} else if (ca->delta_segments > 0) {
		/* In the previous round, we didn't send the entire burst.
		 * Probably some adjustment is needed.
		 */
		DBG("%u [wavetcp_timer_expired] WARNING !!! Not ALL burst sent out\n",
		    tcp_time_stamp);
		BUG_ON(1 == 0);
	} else {
		/* No delta segments, in the previous round we were good */
		ca->delta_segments += current_burst;
	}

	if (current_burst < min_burst) {
		DBG("%u [wavetcp_timer_expired] WARNING !! not min_burst",
		    tcp_time_stamp);
		ca->delta_segments += min_burst - current_burst;
		current_burst = min_burst;
	}

	tp->snd_cwnd += current_burst;

	wavetcp_insert_burst(ca, current_burst);
	wavetcp_history(ca);

	DBG("%u [wavetcp_timer_expired], increased window of %u segments, "
	    "total %u, delta %i, in_flight %u\n",
	    tcp_time_stamp, ca->burst, tp->snd_cwnd, ca->delta_segments,
	    tcp_packets_in_flight(tp));

	BUG_ON(tp->snd_cwnd - tcp_packets_in_flight(tp) > current_burst);

}

/* The TCP is asking for a timer value in jiffies. This will be subject to
 * change for a realtime timer in the future.
 */
static unsigned long wavetcp_get_timer(struct sock *sk)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	BUG_ON(!ca->initialized);

	DBG("%u [wavetcp_get_timer] returning timer of %u ms\n", tcp_time_stamp,
	    ca->tx_timer);

	return msecs_to_jiffies(ca->tx_timer);
}

static void wavetcp_segment_sent(struct sock *sk, u32 sent)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	ca->delta_segments -= sent;

	DBG("%u [wavetcp_segment_sent] sent %u delta %i \n", tcp_time_stamp,
	    sent, ca->delta_segments);
}

static void wavetcp_no_data(struct sock *sk)
{
	wavetcp_reset(sk);
}

static struct tcp_congestion_ops wave_cong_tcp __read_mostly = {
	.init			= wavetcp_init,
	.release		= wavetcp_release,
	.ssthresh		= wavetcp_recalc_ssthresh,
/*	.cong_avoid		= wavetcp_cong_avoid, */
	.cong_control		= wavetcp_cong_control,
	.set_state		= wavetcp_state,
	.undo_cwnd		= wavetcp_undo_cwnd,
	.cwnd_event		= wavetcp_cwnd_event,
	.pkts_acked		= wavetcp_acked,
	.owner			= THIS_MODULE,
	.name			= "wave",
	/* pff name too long. */
	.get_send_timer_exp_time= wavetcp_get_timer,
	.send_timer_expired	= wavetcp_timer_expired,
	.no_data_to_transmit	= wavetcp_no_data,
	.segment_sent		= wavetcp_segment_sent,
};

static int __init wavetcp_register(void)
{
	BUILD_BUG_ON(sizeof(struct wavetcp) > ICSK_CA_PRIV_SIZE);

	return tcp_register_congestion_control(&wave_cong_tcp);
}

static void __exit wavetcp_unregister(void)
{
	tcp_unregister_congestion_control(&wave_cong_tcp);
}

module_init(wavetcp_register);
module_exit(wavetcp_unregister);

MODULE_AUTHOR("Natale Patriciello");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WAVE TCP");
MODULE_VERSION("0.1");
