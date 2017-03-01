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

/* Shift factor for the exponentially weighted average. */
#define SHIFT_FACTOR 10

/* Tell if the driver is initialized (init has been called) */
#define FLAG_INIT	0x1
/* Tell if, as sender, the driver is started (after TX_START) */
#define FLAG_START	0x2
/* If it's true, we save the sent size as a burst */
#define FLAG_SAVE	0x4

/* List for saving the size of sent burst over time */
struct wavetcp_burst_hist {
	u16 size;		/* The burst size */
	struct list_head list;	/* Kernel list declaration */
};

static __always_inline bool test_flag (u8 value, const u8 *flags)
{
	return (*flags & value) == value;
}

static __always_inline void set_flag (u8 value, u8 *flags)
{
	*flags |= value;
}

static __always_inline void clear_flag (u8 value, u8 *flags)
{
	*flags &= ~(value);
}

/* TCP Wave private struct */
struct wavetcp {
	/* The module flags */
	u8 flags;
	/* The current transmission timer (ms) */
	u16 tx_timer;
	/* The current burst size (segments) */
	u16 burst;
	/* Represents a delta from the burst size of segments sent */
	char delta_segments;
	/* The segments acked in the round */
	u32 pkts_acked;
	/* The number of samples to ignore at the beginning of the round. Set
	 * if, for some reason, the TCP send segments out of the timer */
	u8 rtt_samples_to_ignore;
	/* Average ack train dispersion */
	unsigned long avg_ack_train_disp;

	/* First ACK time of the round */
	unsigned int first_ack_time;
	/* First RTT of the round */
	u32 first_rtt;
	/* Minimum RTT of the round */
	u32 min_rtt;
	/* Average RTT of the previous round */
	u32 avg_rtt;
	/* Maximum RTT */
	u32 max_rtt;
	/* Stability factor */
	u8 stab_factor;

	/* The memory cache for saving the burst sizes */
	struct kmem_cache *cache;
	/* The burst history */
	struct wavetcp_burst_hist *history;
};

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

	/* Used to avoid to take the SYN-ACK measurements, and for nothing else */
	ca->flags = 0;
	ca->flags = FLAG_INIT | FLAG_SAVE;

	ca->burst = init_burst;
	ca->delta_segments = init_burst;
	ca->tx_timer = init_timer_ms;
	ca->first_ack_time = 0;
	ca->first_rtt = 0;
	ca->min_rtt = -1; /* a lot of time */
	ca->avg_rtt = 0;
	ca->rtt_samples_to_ignore = 0;
	ca->max_rtt = 0;
	ca->stab_factor = 0;
	ca->avg_ack_train_disp = 0;

	ca->history = kmalloc(sizeof (struct wavetcp_burst_hist), GFP_KERNEL);

	/* Init the history of bwnd */
	INIT_LIST_HEAD(&ca->history->list);

	/* Init our cache pool for the bwnd history */
	ca->cache = KMEM_CACHE(wavetcp_burst_hist, 0);
	BUG_ON(ca->cache == 0);
}

static void wavetcp_release(struct sock *sk)
{
	struct wavetcp *ca = inet_csk_ca(sk);
	struct wavetcp_burst_hist *tmp;
	struct list_head *pos, *q;

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	DBG("%u [wavetcp_release]\n", tcp_time_stamp);

	list_for_each_safe(pos, q, &ca->history->list){
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);
		list_del(pos);
		kmem_cache_free(ca->cache, tmp);
	}

	if (ca->history != 0)
		kfree(ca->history);

	/* Thanks for the cache, we don't need it anymore */
	if (ca->cache != 0)
		kmem_cache_destroy(ca->cache);
}

static void wavetcp_print_history(struct wavetcp *ca)
{
	struct wavetcp_burst_hist *tmp;
	struct list_head *pos, *q;

	list_for_each_safe(pos, q, &ca->history->list){
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);
		DBG("[wavetcp_print_history] %u\n", tmp->size);
	}
}

/* Please explain that we will be forever in congestion avoidance. */
static u32 wavetcp_recalc_ssthresh(struct sock *sk)
{
	DBG("%u [wavetcp_recalc_ssthresh]\n", tcp_time_stamp);
	return 0;
}

static void wavetcp_cong_control(struct sock *sk, const struct rate_sample *rs)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	DBG("%u [wavetcp_cong_control] prior_delivered %u, delivered %i, interval_us %li, "
	    "rtt_us %li, losses %i, ack_sack %u, prior_in_flight %u, is_app %i,"
	    " is_retrans %i\n", tcp_time_stamp, rs->prior_delivered,
	    rs->delivered, rs->interval_us, rs->rtt_us, rs->losses,
	    rs->acked_sacked, rs->prior_in_flight, rs->is_app_limited,
	    rs->is_retrans);

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;
}

static void wavetcp_state(struct sock *sk, u8 new_state)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	switch (new_state) {
	case TCP_CA_Open:
		DBG("%u [wavetcp_state] set CA_Open\n", tcp_time_stamp);
		/* We have fully recovered, so reset some variables */
		ca->rtt_samples_to_ignore = 0;
		ca->delta_segments = 0;
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

	DBG("%u [wavetcp_insert_burst] adding %u segment in the history of burst\n",
	    tcp_time_stamp, burst);

	/* Take the memory from the pre-allocated pool */
	cur = (struct wavetcp_burst_hist *)kmem_cache_alloc(ca->cache,
							    GFP_KERNEL);
	BUG_ON(!cur);
	BUG_ON(burst > init_burst+5);

	cur->size = burst;
	list_add_tail(&cur->list, &ca->history->list);
}

static void wavetcp_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	if(!test_flag(FLAG_INIT, &ca->flags))
		return;

	switch (event) {
	case CA_EVENT_TX_START:
		/* first transmit when no packets in flight */
		DBG("%u [wavetcp_cwnd_event] TX_START\n", tcp_time_stamp);

		if (!test_flag(FLAG_START, &ca->flags))
			wavetcp_insert_burst(ca, init_burst);

		set_flag(FLAG_START, &ca->flags);

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

static __always_inline void wavetcp_adj_mode(struct wavetcp *ca,
					     unsigned long ack_train_disp,
					     unsigned long delta_rtt)
{
	ca->stab_factor = ca->avg_rtt / ack_train_disp;

	ca->min_rtt = 0;
	ca->avg_rtt = ca->max_rtt;
	ca->tx_timer = init_timer_ms;

	DBG("%u [wavetcp_adj_mode] stab_factor %u, avg_rtt %u, timer %u ms",
	    tcp_time_stamp, ca->stab_factor, ca->avg_rtt, ca->tx_timer);
}

static __always_inline void wavetcp_tracking_mode(struct wavetcp *ca,
						  unsigned int ack_train_disp,
						  unsigned long delta_rtt)
{
	ca->tx_timer = (ack_train_disp + (delta_rtt / 2)) / 1000;

	if (ca->tx_timer == 0) {
		DBG("%u [wavetcp_tracking_mode] increasing tx timer to 1 ms",
		    tcp_time_stamp);
		ca->tx_timer = 1;
	}

	DBG("%u [wavetcp_tracking_mode] tx timer is %u ms", tcp_time_stamp,
	    ca->tx_timer);
}

/* The weight a is:
 *
 * a = (first_rtt - min_rtt) / first_rtt
 *
 * we scale the first substraction by 1024 (to have a precision up to a = 0.001)
 */
static __always_inline unsigned long wavetcp_compute_weight(unsigned long first_rtt,
							    unsigned long min_rtt)
{
	unsigned long diff = first_rtt - min_rtt;
	diff = diff << SHIFT_FACTOR;

	return diff / first_rtt;
}

static void wavetcp_round_terminated(struct wavetcp *ca, u32 burst)
{
	unsigned long delta_rtt;
	unsigned int ack_train_disp;
	DBG("%u [wavetcp_round_terminated] reached the burst size %u\n",
	    tcp_time_stamp, burst);

	BUG_ON(time_after((unsigned long) ca->first_ack_time,
			  (unsigned long) tcp_time_stamp));

	/* Why the first if?
	 *
	 * a = (first_rtt - min_rtt) / first_rtt = 1 - (min_rtt/first_rtt)
	 *
	 * avg_rtt_0 = (1 - a) * first_rtt
	 *           = (1 - (1 - (min_rtt/first_rtt))) * first_rtt
	 *           = first_rtt - (first_rtt - min_rtt)
	 *           = min_rtt
	 *
	 *
	 * And.. what happen in the else branch? We calculate first a (scaled by
	 * 1024), then do the substraction (1-a) by keeping in the consideration
	 * the scale, and in the end coming back to the result removing the
	 * scaling.
	 *
	 * We divide the equation
	 *
	 * AvgRtt = a * AvgRtt + (1-a)*Rtt
	 *
	 * in two part properly scaled, left and right, and then having a sum of
	 * the two parts to avoid (possible) overflow.
	 */
	if (ca->avg_rtt == 0)
		ca->avg_rtt = ca->min_rtt;
	else {
		unsigned long a;
		unsigned long left;
		unsigned long right;
		a = wavetcp_compute_weight(ca->first_rtt, ca->min_rtt);

		DBG("%u [wavetcp_round_terminated] init. avg %u us, first %u us, "
		    "min %u us, a (shifted) %lu",
		    tcp_time_stamp, ca->avg_rtt, ca->first_rtt, ca->min_rtt, a);

		left = (a * ca->avg_rtt) >> SHIFT_FACTOR;
		right = (((1 << SHIFT_FACTOR)-a) * ca->first_rtt) >> SHIFT_FACTOR;

		ca->avg_rtt = left + right;
	}

	/* We clearly missed a measurements if this happens */
	BUG_ON(ca->avg_rtt < ca->min_rtt);

	ack_train_disp = jiffies_to_usecs(tcp_time_stamp - ca->first_ack_time);
	delta_rtt = ca->avg_rtt - ca->min_rtt;

	/* Take the average train dispersion with a weighted average (factor?) */
	if (ca->avg_ack_train_disp == 0)
		ca->avg_ack_train_disp = ack_train_disp;
	else if (ack_train_disp != 0)
		ca->avg_ack_train_disp = (ca->avg_ack_train_disp/2) + (ack_train_disp/2);
	else if (ack_train_disp == 0)
		/* We received a cumulative ACK just after we sent the data, so
		 * the dispersion would be close to zero. Just use the average
		 * in this case */
		ack_train_disp = ca->avg_ack_train_disp;

	DBG("%u [wavetcp_round_terminated] done. ack_train_disp %u us delta_rtt %lu us "
	    "avg_rtt %u us, sf %u\n", tcp_time_stamp,
	    ack_train_disp, delta_rtt, ca->avg_rtt, ca->stab_factor);

	if (ca->stab_factor > 0) {
		--ca->stab_factor;
		return;
	}

	/* delta_rtt is in us, beta_ms in ms */
	if (delta_rtt > beta_ms * 1000)
		wavetcp_adj_mode(ca, ack_train_disp, delta_rtt);
	else
		wavetcp_tracking_mode(ca, ack_train_disp, delta_rtt);
}

static void wavetcp_acce(struct wavetcp *ca, s32 rtt_us, u32 pkts_acked)
{
	struct wavetcp_burst_hist *tmp;
	struct list_head *pos;

	pos = ca->history->list.next;
	tmp = list_entry(pos, struct wavetcp_burst_hist, list);

	if (tmp->size == 0) {
		/* No burst in memory. Most likely we sent some segments out of
		 * the allowed window (e.g., loss probe) */
		DBG("%u [wavetcp_acce] WARNING! empty burst\n", tcp_time_stamp);
		wavetcp_print_history(ca);
		goto reset;
	}

	if (ca->first_ack_time == 0) {
		ca->first_ack_time = tcp_time_stamp;
		DBG("%u [wavetcp_acce] first ack of the train\n",
		    tcp_time_stamp);
	}

	if (ca->first_rtt == 0) {
		/* If this is the first sample, take the RTT and remember the time.
		 * Small filtering: avoid to init the round if the rtt is very high.
		 * Added to avoid to begin the measurement after a tail loss probe, so
		* probably it could be the case to add a test over the ca state. */
		if (rtt_us > 3*ca->avg_rtt && ca->rtt_samples_to_ignore > 0) {
			DBG("%u [wavetcp_acce] Ignoring first rtt of %i\n",
			    tcp_time_stamp, rtt_us);
			ca->rtt_samples_to_ignore--;
			return;
		}

		if (rtt_us > 0)
			ca->first_rtt = rtt_us;
		else
			/* We don't have the first RTT of the burst. TODO: Maybe
			 * try to get the 2nd, the 3rd, ..., and only if they
			 * are all 0 then use the avg? */
			ca->first_rtt = ca->avg_rtt;

		DBG("%u [wavetcp_acce] first measurement rtt %i\n",
		    tcp_time_stamp, ca->first_rtt);
	}

	/* Check the minimum rtt we have seen */
	if (rtt_us > 0 && rtt_us < ca->min_rtt) {
		ca->min_rtt = rtt_us;
		DBG("%u [wavetcp_acce] min rtt %u\n", tcp_time_stamp,
		    rtt_us);
	}

	if (rtt_us > ca->max_rtt)
		ca->max_rtt = rtt_us;

	/* Train management. Pkts_acked could be 0 */
	ca->pkts_acked += pkts_acked;

	if (ca->pkts_acked < tmp->size)
		return;

	while (ca->pkts_acked >= tmp->size) {
		wavetcp_round_terminated(ca, tmp->size);

		ca->pkts_acked -= tmp->size;

		DBG("%u [wavetcp_acce] resetting RTT values for next round\n",
		    tcp_time_stamp);

		/* Delete the burst from the history */
		list_del(pos);
		kmem_cache_free(ca->cache, tmp);

		/* Take next burst */
		pos = ca->history->list.next;
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);
	}

reset:
	/* Reset the variables needed for the beginning of the next round*/
	ca->first_ack_time = 0;
	ca->first_rtt = 0;
	DBG("%u [wavetcp_acce] resetting RTT values for next round\n",
	    tcp_time_stamp);
}

/* Invoked each time we receive an ACK. Obviously, this function also gets
 * called when we receive the SYN-ACK, but we ignore it thanks to the
 * FLAG_INIT flag.
 *
 * We close the cwnd of the amount of segments acked, because we don't like
 * sending out segments if the timer is not expired. Without doing this, we
 * would end with cwnd - in_flight > 0.
 */
static void wavetcp_acked(struct sock *sk, const struct ack_sample *sample)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);
	s32 rtt_us;

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	DBG("%u [wavetcp_acked] pkts_acked %u, rtt_us %i, in_flight %u "
	    ", cwnd %u, seq ack %u\n",
	    tcp_time_stamp, sample->pkts_acked, sample->rtt_us,
	    sample->in_flight, tp->snd_cwnd, tp->rtt_seq);

	/* More than 2 is probably a cumulative ACK after a retransmit phase.
	 * Use the average RTT */
	if (sample->pkts_acked > 2)
		rtt_us = (s32) ca->avg_rtt;
	else
		rtt_us = sample->rtt_us;

	/* We can divide the ACCE function in two part: the first take care of
	 * the RTT, and the second of the train management. Here we could have
	 * pkts_acked == 0, but with RTT values (because the underlying TCP can
	 * identify what segment has been ACKed through the SACK option). In any
	 * case, therefore, we enter wavetcp_acce.*/
	wavetcp_acce (ca, rtt_us, sample->pkts_acked);

	if (tp->snd_cwnd < sample->pkts_acked) {
		/* We sent some scattered segments, so the burst segments and
		 * the ACK we get is not aligned.
		 */
		ca->delta_segments += sample->pkts_acked - tp->snd_cwnd;
	}

	/* Brutally set the cwnd in order to not let segment out */
	tp->snd_cwnd = tcp_packets_in_flight(tp);

	DBG("%u [wavetcp_acked] new window %u in_flight %u delta %i\n",
	    tcp_time_stamp, tp->snd_cwnd, tcp_packets_in_flight(tp),
	    ca->delta_segments);
}

/* The TCP informs us that the timer is expired (or has never been set). We can
 * infer the latter by the FLAG_STARTED flag: if it's false, don't increase the
 * cwnd, because it is at its default value (init_burst) and we still have to
 * transmit the first burst.
 */
static void wavetcp_timer_expired(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);
	u32 current_burst = ca->burst;

	BUG_ON(!test_flag(FLAG_INIT, &ca->flags));

	if (!test_flag(FLAG_START, &ca->flags))
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
		 * Probably some adjustment is needed. But for now nothing.
		 */
		ca->delta_segments += current_burst;
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
	set_flag(FLAG_SAVE, &ca->flags);

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
	u16 timer;

	BUG_ON(!test_flag(FLAG_INIT, &ca->flags));

	timer = min_t(unsigned long, ca->tx_timer, init_timer_ms);

	DBG("%u [wavetcp_get_timer] returning timer of %u ms\n", tcp_time_stamp,
	    timer);

	return msecs_to_jiffies(timer);
}

static u64 wavetcp_rate_bytes_per_sec(struct sock *sk)
{
	struct wavetcp *ca = inet_csk_ca(sk);
	u64 rate;
	u64 times_per_sec;

	BUG_ON(ca->tx_timer > MSEC_PER_SEC);

	times_per_sec = MSEC_PER_SEC / ca->tx_timer;

	rate = ca->burst;
	rate *= times_per_sec;
	rate *= tcp_mss_to_mtu(sk, tcp_sk(sk)->mss_cache);

	DBG("%u [wavetcp_rate_bytes_per_sec] calculated a rate of %llu B/s\n",
	    tcp_time_stamp, rate);

	return rate;
}

static void wavetcp_segment_sent(struct sock *sk, u32 sent)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);
	u64 rate;

	BUG_ON(sent > ca->burst);

	if (ca->delta_segments < sent)
		ca->rtt_samples_to_ignore += sent;

	ca->delta_segments -= sent;

	if (test_flag(FLAG_SAVE, &ca->flags) && sent > 0)
		wavetcp_insert_burst(ca, sent);

	clear_flag(FLAG_SAVE, &ca->flags);

	if (ca->delta_segments >= 0 &&
	    ca->burst > sent &&
	    tcp_packets_in_flight(tp) <= tp->snd_cwnd) {
		/* Reduce the cwnd accordingly, because we didn't sent enough
		 * to cover it (we are app limited probably) */
		tp->snd_cwnd -= ca->burst - sent;
		DBG("%u [wavetcp_segment_sent] reducing cwnd by %u\n",
		    tcp_time_stamp, ca->burst - sent);
	}

	rate = wavetcp_rate_bytes_per_sec (sk);

	DBG("%u [wavetcp_segment_sent] sent %u delta %i rate %llu \n", tcp_time_stamp,
	    sent, ca->delta_segments, rate);
}

static void wavetcp_no_data(struct sock *sk)
{
	DBG("%u [wavetcp_no_data]\n", tcp_time_stamp);
}

static u32 wavetcp_sndbuf_expand(struct sock *sk)
{
	/* We need that data. TODO: Check from the rate and the mss how many
	 * packets it is safe to store. */
	return 10;
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
	.sndbuf_expand		= wavetcp_sndbuf_expand,
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
