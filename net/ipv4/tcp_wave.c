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
 */

#include <net/tcp.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define WAVE_DEBUG 1

#ifdef WAVE_DEBUG
	#define DBG(msg ...) printk(KERN_DEBUG "WAVE_DEBUG: " msg)
#else
	static inline void DBG(const char *msg, ...) { }
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
#define AVG_SCALE 20
#define AVG_UNIT (1 << AVG_SCALE)

/* Taken from BBR */
#define BW_SCALE 24
#define BW_UNIT (1 << BW_SCALE)

/* Tell if the driver is initialized (init has been called) */
#define FLAG_INIT       0x1
/* Tell if, as sender, the driver is started (after TX_START) */
#define FLAG_START      0x2
/* If it's true, we save the sent size as a burst */
#define FLAG_SAVE       0x4

/* List for saving the size of sent burst over time */
struct wavetcp_burst_hist {
	u16 size;               /* The burst size */
	struct list_head list;  /* Kernel list declaration */
};

static __always_inline bool test_flag(u8 value, const u8 *flags)
{
	return (*flags & value) == value;
}

static __always_inline void set_flag(u8 value, u8 *flags)
{
	*flags |= value;
}

static __always_inline void clear_flag(u8 value, u8 *flags)
{
	*flags &= ~(value);
}

/* TCP Wave private struct */
struct wavetcp {
	/* The module flags */
	u8 flags;
	/* The current transmission timer (us) */
	u32 tx_timer;
	/* The current burst size (segments) */
	u16 burst;
	/* Represents a delta from the burst size of segments sent */
	char delta_segments;
	/* The segments acked in the round */
	u16 pkts_acked;
	/* Heuristic scale, to divide the RTT */
	u8 heuristic_scale;
	/* Previous ack_train_disp Value */
	u32 previous_ack_train_disp;
	/* First ACK time of the round */
	u32 first_ack_time;
	/* Backup value of the first ack time */
	u32 backup_first_ack_time;
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
	/* To Print TCP Source Port  */
	u16 sport;
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

	ca->sport = ntohs(inet_sk(sk)->inet_sport);

	DBG("%u sport: %u [%s]\n", tcp_time_stamp, ca->sport,
	    __func__);

	/* Setting the initial Cwnd to 0 will not call the TX_START event */
	tp->snd_ssthresh = 0;
	tp->snd_cwnd = init_burst;

	/* Used to avoid to take the SYN-ACK measurements */
	ca->flags = 0;
	ca->flags = FLAG_INIT | FLAG_SAVE;

	ca->burst = init_burst;
	ca->delta_segments = init_burst;
	ca->tx_timer = init_timer_ms * USEC_PER_MSEC;
	ca->first_ack_time = 0;
	ca->backup_first_ack_time = 0;
	ca->heuristic_scale = 0;
	ca->first_rtt = 0;
	ca->min_rtt = -1; /* a lot of time */
	ca->avg_rtt = 0;
	ca->max_rtt = 0;
	ca->stab_factor = 0;
	ca->previous_ack_train_disp = 0;

	ca->history = kmalloc(sizeof(*ca->history), GFP_KERNEL);

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

	DBG("%u sport: %u [%s]\n", tcp_time_stamp, ca->sport,
	    __func__);

	list_for_each_safe(pos, q, &ca->history->list) {
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

	list_for_each_safe(pos, q, &ca->history->list) {
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);
		DBG("[%s] %u\n", __func__, tmp->size);
	}
}

/* Please explain that we will be forever in congestion avoidance. */
static u32 wavetcp_recalc_ssthresh(struct sock *sk)
{
	DBG("%u [%s]\n", tcp_time_stamp, __func__);
	return 0;
}

static void wavetcp_state(struct sock *sk, u8 new_state)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	switch (new_state) {
	case TCP_CA_Open:
		DBG("%u sport: %u [%s] set CA_Open\n", tcp_time_stamp,
		    ca->sport, __func__);
		/* We have fully recovered, so reset some variables */
		ca->delta_segments = 0;
		break;
	default:
		DBG("%u sport: %u [%s] set state %u, ignored\n",
		    tcp_time_stamp, ca->sport, __func__, new_state);
	}
}

static u32 wavetcp_undo_cwnd(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);

	/* Not implemented yet. We stick to the decision made earlier */
	DBG("%u [%s]\n", tcp_time_stamp, __func__);
	return tp->snd_cwnd;
}

/* Add the size of the burst in the history of bursts */
static void wavetcp_insert_burst(struct wavetcp *ca, u32 burst)
{
	struct wavetcp_burst_hist *cur;

	DBG("%u sport: %u [%s] adding %u segment in the history of burst\n",
	    tcp_time_stamp, ca->sport, __func__, burst);

	/* Take the memory from the pre-allocated pool */
	cur = (struct wavetcp_burst_hist *)kmem_cache_alloc(ca->cache,
							    GFP_KERNEL);
	BUG_ON(!cur);

	cur->size = burst;
	list_add_tail(&cur->list, &ca->history->list);
}

static void wavetcp_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct wavetcp *ca = inet_csk_ca(sk);

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	switch (event) {
	case CA_EVENT_TX_START:
		/* first transmit when no packets in flight */
		DBG("%u sport: %u [%s] TX_START\n", tcp_time_stamp,
		    ca->sport, __func__);

		set_flag(FLAG_START, &ca->flags);

		break;
	default:
		DBG("%u sport: %u [%s] got event %u, ignored\n",
		    tcp_time_stamp, ca->sport, __func__, event);
		break;
	}
}

static __always_inline void wavetcp_adj_mode(struct wavetcp *ca,
					     unsigned long delta_rtt)
{
	ca->stab_factor = ca->avg_rtt / ca->tx_timer;

	ca->min_rtt = -1; /* a lot of time */
	ca->avg_rtt = ca->max_rtt;
	ca->tx_timer = init_timer_ms * USEC_PER_MSEC;

	DBG("%u sport: %u [%s] stab_factor %u, timer %u us, avg_rtt %u us\n",
	    tcp_time_stamp, ca->sport, __func__, ca->stab_factor,
	    ca->tx_timer, ca->avg_rtt);
}

static __always_inline void wavetcp_tracking_mode(struct wavetcp *ca,
						  u32 ack_train_disp,
						  u64 delta_rtt)
{
	if (ack_train_disp == 0) {
		DBG("%u sport: %u [%s] ack_train_disp is 0. Impossible to do tracking.\n",
		    tcp_time_stamp, ca->sport, __func__);
		return;
	}

	ca->tx_timer = (ack_train_disp + (delta_rtt / 2));

	if (ca->tx_timer == 0) {
		DBG("%u sport: %u [%s] WARNING: tx timer is 0"
		    ", forcefully set it to 1000 us\n",
		    tcp_time_stamp, ca->sport, __func__);
		ca->tx_timer = 1000;
	}

	DBG("%u sport: %u [%s] tx timer is %u us\n",
	    tcp_time_stamp, ca->sport, __func__,
	    ca->tx_timer);
}

/* The weight a is:
 *
 * a = (first_rtt - min_rtt) / first_rtt
 *
 */
static __always_inline u64 wavetcp_compute_weight(u32 first_rtt,
						  u32 min_rtt)
{
	u64 diff = first_rtt - min_rtt;

	diff = diff * AVG_UNIT;

	return diff / first_rtt;
}

static u32 heuristic_ack_train_disp(struct wavetcp *ca, const struct rate_sample *rs,
				    u32 burst)
{
	u32 ack_train_disp = 0;
	u32 backup_interval = 0;

	BUG_ON (ca->previous_ack_train_disp != 0);

	/*
	 * The heuristic takes the RTT of the first ACK, the RTT of the
	 * latest ACK, and uses the difference as ack_train_disp.
	 *
	 * If the sample for the first and last ACK are the same (e.g.,
	 * one ACK per burst) we use as the latest option the value of
	 * interval_us (which is the RTT). However, this value is
	 * exponentially lowered each time we don't have any valid
	 * sample (i.e., we perform a division by 2, by 4, and so on).
	 * The increased transmitted rate, if it is out of the capacity
	 * of the bottleneck, will be compensated by an higher
	 * delta_rtt, and so limited by the adjustment algorithm. This
	 * is a blind search, but we do not have any valid sample...
	 */
	if (rs->interval_us > 0) {
		if (rs->interval_us >= ca->backup_first_ack_time) {
			/* first heuristic */
			backup_interval = rs->interval_us - ca->backup_first_ack_time;
		} else {
			/* this branch avoids an overflow. However, reaching
			 * this point means that the ACK train is not aligned
			 * with the sent burst.
			 */
			backup_interval = ca->backup_first_ack_time - rs->interval_us;
		}

		if (backup_interval == 0) {
			/* Blind search */
			ack_train_disp = rs->interval_us >> ca->heuristic_scale;
			++ca->heuristic_scale;
			DBG("%u sport: %u [%s] we received one BIG ack."
			    " Doing an heuristic with scale %u, interval_us"
			    " %li us, and setting ack_train_disp to %u us\n",
			    tcp_time_stamp, ca->sport, __func__,
			    ca->heuristic_scale, rs->interval_us, ack_train_disp);
		} else {
			ack_train_disp = backup_interval;
			DBG("%u sport: %u [%s] we got the first ack with"
			    " interval %u us, the last (this) with interval %li us."
			    " Doing a substraction and setting ack_train_disp"
			    " to %u us\n",
			    tcp_time_stamp, ca->sport, __func__,
			    ca->backup_first_ack_time, rs->interval_us,
			    ack_train_disp);
		}
	} else {
		DBG("%u sport: %u [%s] WARNING is not possible "
		    "to heuristically calculate ack_train_disp, returning 0."
		    "Delivered %u, interval_us %li\n",
		    tcp_time_stamp, ca->sport, __func__,
		    rs->delivered, rs->interval_us);
		return 0;
	}

	return ack_train_disp;
}

static u32 calculate_ack_train_disp(struct wavetcp *ca,
				    const struct rate_sample *rs,
				    u32 burst, u64 delta_rtt)
{
	u32 ack_train_disp = jiffies_to_usecs(tcp_time_stamp - ca->first_ack_time);

	if (ca->previous_ack_train_disp == 0 && ack_train_disp == 0) {
		/* We received a cumulative ACK just after we sent the data, so
		 * the dispersion would be close to zero, OR the connection
		 * is so fast that tcp_time_stamp is not good enough to measure
		 * time. Moreover, we don't have any valid sample from the past;
		 * in this case, we use an heuristic to calculate
		 * ack_train_disp.
		 */
		return heuristic_ack_train_disp(ca, rs, burst);
	}

	DBG("%u sport: %u [%s] using measured ack_train_disp %u",
	    tcp_time_stamp, ca->sport, __func__, ack_train_disp);

	/* resetting the heuristic scale because we have a real sample */
	ca->heuristic_scale = 0;

	if (ca->previous_ack_train_disp == 0) {
		/* initialize the value */
		ca->previous_ack_train_disp = ack_train_disp;
	} else if (ack_train_disp > ca->previous_ack_train_disp) {
		/* filter the measured value */
		u64 alpha;
		u64 left;
		u64 right;

		alpha = (delta_rtt * AVG_UNIT) / (beta_ms * 1000);
		left = ((AVG_UNIT - alpha) * ca->previous_ack_train_disp) / AVG_UNIT;
		right = (alpha * ack_train_disp) / AVG_UNIT;
		DBG("%u sport: %u [%s] AVG_UNIT %i delta_rtt %llu beta %i alpha %llu "
		    "rcv_ack_train_disp %u prv_ack_train_disp %u left %llu right %llu\n",
		    tcp_time_stamp, ca->sport, __func__, AVG_UNIT, delta_rtt,
		    beta_ms, alpha, ack_train_disp, ca->previous_ack_train_disp,
		    left, right);

		ack_train_disp = (u32)left + (u32)right;

		DBG("%u sport: %u [%s] filtered_ack_train_disp %u (u32)left %u (u32)right %u\n",
		    tcp_time_stamp, ca->sport, __func__, ack_train_disp,
		    (u32)left, (u32)right);

	} else if (ack_train_disp == 0) {
		/* Use the plain previous value */
		ack_train_disp = ca->previous_ack_train_disp;
	} else {
		/* In all other cases, update the previous value */
		ca->previous_ack_train_disp = ack_train_disp;
	}

	DBG("%u sport: %u [%s] previous_ack_train_disp %u us, final ack_train_disp %u us\n",
	    tcp_time_stamp, ca->sport, __func__,
	    ca->previous_ack_train_disp, ack_train_disp);

	return ack_train_disp;
}

static u64 calculate_delta_rtt(struct wavetcp *ca)
{
	if (ca->first_rtt == 0) {
		ca->first_rtt = ca->avg_rtt;
		DBG("%u sport: %u [%s] It was impossible to get any rtt "
		    "in the train. Using the average value %u\n",
		    tcp_time_stamp, ca->sport, __func__,
		    ca->first_rtt);
	}
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
	if (ca->avg_rtt == 0) {
		ca->avg_rtt = ca->min_rtt;
	} else if (ca->first_rtt > 0) {
		u64 a;
		u64 left;
		u64 right;
		a = wavetcp_compute_weight(ca->first_rtt, ca->min_rtt);

		DBG("%u sport: %u [%s] init. avg %u us, first %u us, "
		    "min %u us, a (shifted) %llu",
		    tcp_time_stamp, ca->sport, __func__,
		    ca->avg_rtt, ca->first_rtt, ca->min_rtt, a);

		left = (a * ca->avg_rtt) / AVG_UNIT;
		right = ((AVG_UNIT - a) * ca->first_rtt) / AVG_UNIT;

		ca->avg_rtt = (u32)left + (u32)right;
	} else {
		DBG("%u sport: %u [%s] first_rtt is 0. It is impossible "
		    "to calculate the average RTT. Using the old value.\n",
		    tcp_time_stamp, ca->sport, __func__);
	}

	DBG("%u sport: %u [%s] final avg %u\n",
	    tcp_time_stamp, ca->sport, __func__, ca->avg_rtt);
	/* We clearly missed a measurements if this happens */
	BUG_ON(ca->avg_rtt < ca->min_rtt);
	return ca->avg_rtt - ca->min_rtt;
}

static void wavetcp_round_terminated(struct sock *sk, const struct rate_sample *rs,
				     u32 burst)
{
	u64 delta_rtt;
	struct wavetcp *ca = inet_csk_ca(sk);

	DBG("%u sport: %u [%s] reached the burst size %u\n",
	    tcp_time_stamp, ca->sport, __func__, burst);

	BUG_ON(time_after((unsigned long)ca->first_ack_time,
			  (unsigned long)tcp_time_stamp));

	delta_rtt = calculate_delta_rtt(ca);
	DBG("%u sport: %u [%s] delta rtt %llu us\n",
	    tcp_time_stamp, ca->sport, __func__, delta_rtt);

	/* If we have to wait, let's wait */
	if (ca->stab_factor > 0) {
		--ca->stab_factor;
		DBG("%u sport: %u [%s] avoiding update for stability reasons\n",
		    tcp_time_stamp, ca->sport, __func__);
		return;
	}

	DBG("%u sport: %u [%s] drtt %llu\n",
	    tcp_time_stamp, ca->sport, __func__, delta_rtt);

	/* delta_rtt is in us, beta_ms in ms */
	if (delta_rtt > beta_ms * 1000)
		wavetcp_adj_mode(ca,  delta_rtt);
	else
		wavetcp_tracking_mode(ca, calculate_ack_train_disp(ca, rs,
								   burst,
								   delta_rtt),
				      delta_rtt);
}

static void wavetcp_cong_control(struct sock *sk, const struct rate_sample *rs)
{
	struct wavetcp_burst_hist *tmp;
	struct list_head *pos;
	struct wavetcp *ca = inet_csk_ca(sk);

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	if (ca->backup_first_ack_time == 0 && rs->interval_us > 0)
		ca->backup_first_ack_time = rs->interval_us;

	pos = ca->history->list.next;
	tmp = list_entry(pos, struct wavetcp_burst_hist, list);

	if (tmp->size == 0) {
		/* No burst in memory. Most likely we sent some segments out of
		 * the allowed window (e.g., loss probe) */
		DBG("%u sport: %u [%s] WARNING! empty burst\n",
		    tcp_time_stamp, ca->sport, __func__);
		wavetcp_print_history(ca);
		goto reset;
	}

	DBG("%u sport: %u [%s] prior_delivered %u, delivered %i, interval_us %li, "
	    "rtt_us %li, losses %i, ack_sack %u, prior_in_flight %u, is_app %i,"
	    " is_retrans %i\n", tcp_time_stamp, ca->sport, __func__,
	    rs->prior_delivered, rs->delivered, rs->interval_us, rs->rtt_us,
	    rs->losses, rs->acked_sacked, rs->prior_in_flight,
	    rs->is_app_limited, rs->is_retrans);

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	/* Train management.*/
	ca->pkts_acked += rs->acked_sacked;

	if (ca->pkts_acked < tmp->size)
		return;

	while (ca->pkts_acked >= tmp->size) {
		/* Usually the burst end is also reflected in the rs->delivered
		 * variable. If this is not the case, and such variable is
		 * behind just for 1 segment, then do this experimental thing
		 * to re-allineate the burst with the rs->delivered variable.
		 * In the majority of cases, we went out of allineation because
		 * of a tail loss probe. */
		if (rs->delivered + 1 == tmp->size) {
			DBG("%u sport: %u [%s] highly experimental:"
			    " ignore 1 pkt. pkts_acked %u, delivered %u,"
			    " burst %u\n", tcp_time_stamp, ca->sport, __func__,
			    ca->pkts_acked, rs->delivered, tmp->size);
			ca->pkts_acked--;
			return;
		}
		wavetcp_round_terminated(sk, rs, tmp->size);

		BUG_ON(ca->pkts_acked < tmp->size);

		ca->pkts_acked -= tmp->size;

		/* Delete the burst from the history */
		list_del(pos);
		kmem_cache_free(ca->cache, tmp);

		/* Take next burst */
		pos = ca->history->list.next;
		tmp = list_entry(pos, struct wavetcp_burst_hist, list);

		/* If we cycle, inside wavetcp_round_terminated we will take the
		 * Linux path instead of the wave path.. first_rtt will not be
		 * read, so don't waste a cycle to set it */
		ca->first_ack_time = tcp_time_stamp;
		ca->backup_first_ack_time = 0;
	}

reset:
	/* Reset the variables needed for the beginning of the next round*/
	ca->first_ack_time = 0;
	ca->backup_first_ack_time = 0;
	ca->first_rtt = 0;
	DBG("%u sport: %u [%s] resetting RTT values for next round\n",
	    tcp_time_stamp, ca->sport, __func__);
}

static void wavetcp_acce(struct wavetcp *ca, s32 rtt_us, u32 pkts_acked)
{
	if (ca->first_ack_time == 0) {
		ca->first_ack_time = tcp_time_stamp;
		DBG("%u sport: %u [%s] first ack of the train\n",
		    tcp_time_stamp, ca->sport, __func__);
	}

	if (ca->first_rtt == 0 && rtt_us > 0) {
		ca->first_rtt = rtt_us;

		DBG("%u sport: %u [%s] first measurement rtt %i\n",
		    tcp_time_stamp, ca->sport, __func__,
		    ca->first_rtt);
	}

	if (rtt_us <= 0)
		return;

	/* Check the minimum rtt we have seen */
	if (rtt_us < ca->min_rtt) {
		ca->min_rtt = rtt_us;
		DBG("%u sport: %u [%s] min rtt %u\n", tcp_time_stamp,
		    ca->sport, __func__, rtt_us);
	}

	if (rtt_us > ca->max_rtt)
		ca->max_rtt = rtt_us;
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

	if (!test_flag(FLAG_INIT, &ca->flags))
		return;

	DBG("%u sport: %u [%s] pkts_acked %u, rtt_us %i, in_flight %u "
	    ", cwnd %u, seq ack %u\n",
	    tcp_time_stamp, ca->sport, __func__, sample->pkts_acked,
	    sample->rtt_us, sample->in_flight, tp->snd_cwnd, tp->snd_una);

	/* We can divide the ACCE function in two part: the first take care of
	 * the RTT, and the second of the train management. Here we could have
	 * pkts_acked == 0, but with RTT values (because the underlying TCP can
	 * identify what segment has been ACKed through the SACK option). In any
	 * case, therefore, we enter wavetcp_acce.*/
	wavetcp_acce(ca, sample->rtt_us, sample->pkts_acked);

	if (tp->snd_cwnd < sample->pkts_acked) {
		/* We sent some scattered segments, so the burst segments and
		 * the ACK we get is not aligned.
		 */
		DBG("%u sport: %u [%s] delta_seg %i\n",
		    tcp_time_stamp, ca->sport, __func__,
		    ca->delta_segments);

		ca->delta_segments += sample->pkts_acked - tp->snd_cwnd;
	}

	DBG("%u sport: %u [%s] snd_cwnd %u pkts_acked %u delta %i\n",
	    tcp_time_stamp, ca->sport, __func__, tp->snd_cwnd,
	    sample->pkts_acked, ca->delta_segments);

	/* Brutally set the cwnd in order to not let segment out */
	tp->snd_cwnd = tcp_packets_in_flight(tp);

	DBG("%u sport: %u [%s] new window %u in_flight %u delta %i\n",
	    tcp_time_stamp, ca->sport, __func__, tp->snd_cwnd,
	    tcp_packets_in_flight(tp), ca->delta_segments);
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

	if (!test_flag(FLAG_START, &ca->flags)) {
		DBG("%u sport: %u [%s] returning because of !FLAG_START, leaving cwnd %u\n",
		    tcp_time_stamp, ca->sport, __func__, tp->snd_cwnd);
		return;
	}

	DBG("%u sport: %u [%s] starting with delta %u current_burst %u\n",
	    tcp_time_stamp, ca->sport, __func__, ca->delta_segments,
	    current_burst);

	if (ca->delta_segments < 0) {
		/* In the previous round, we sent more than the allowed burst,
		 * so reduce the current burst.
		 */
		BUG_ON(current_burst > ca->delta_segments);
		current_burst += ca->delta_segments; /* please *reduce* */

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
			DBG("%u sport: %u [%s] adding %u to balance "
			    "segments sent out of window", tcp_time_stamp,
			    ca->sport, __func__, diff);
		}
	}

	ca->delta_segments = current_burst;
	DBG("%u sport: %u [%s] setting delta_seg %u current burst %u\n",
	    tcp_time_stamp, ca->sport, __func__,
	    ca->delta_segments, current_burst);

	if (current_burst < min_burst) {
		DBG("%u sport: %u [%s] WARNING !! not min_burst",
		    tcp_time_stamp, ca->sport, __func__);
		ca->delta_segments += min_burst - current_burst;
		current_burst = min_burst;
	}

	tp->snd_cwnd += current_burst;
	set_flag(FLAG_SAVE, &ca->flags);

	DBG("%u sport: %u [%s], increased window of %u segments, "
	    "total %u, delta %i, in_flight %u\n",
	    tcp_time_stamp, ca->sport, __func__, ca->burst,
	    tp->snd_cwnd, ca->delta_segments, tcp_packets_in_flight(tp));

	if (tp->snd_cwnd - tcp_packets_in_flight(tp) > current_burst) {
		DBG("%u sport: %u [%s] WARNING! "
		    " cwnd %u, in_flight %u, current burst %u\n",
		    tcp_time_stamp, ca->sport, __func__,
		    tp->snd_cwnd, tcp_packets_in_flight(tp),
		    current_burst);
	}
}

/* The TCP is asking for a timer value in jiffies. This will be subject to
 * change for a realtime timer in the future.
 */
static unsigned long wavetcp_get_timer(struct sock *sk)
{
	struct wavetcp *ca = inet_csk_ca(sk);
	u32 timer;

	BUG_ON(!test_flag(FLAG_INIT, &ca->flags));

	timer = min_t(unsigned long, ca->tx_timer, init_timer_ms * USEC_PER_MSEC);

	DBG("%u sport: %u [%s] returning timer of %u us\n",
	    tcp_time_stamp, ca->sport, __func__, timer);

	return usecs_to_jiffies(timer);
}

static void wavetcp_segment_sent(struct sock *sk, u32 sent)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct wavetcp *ca = inet_csk_ca(sk);

	if (test_flag(FLAG_SAVE, &ca->flags) && sent > 0) {
		wavetcp_insert_burst(ca, sent);
		clear_flag(FLAG_SAVE, &ca->flags);
	} else {
		DBG("%u sport: %u [%s] not saving burst, sent %u\n",
		    tcp_time_stamp, ca->sport, __func__, sent);
	}

	if (sent > ca->burst) {
		DBG("%u sport: %u [%s] WARNING! sent %u, burst %u"
		    " cwnd %u delta_seg %i\n, TSO very probable",
		    tcp_time_stamp, ca->sport, __func__, sent,
		    ca->burst, tp->snd_cwnd, ca->delta_segments);
	}

	ca->delta_segments -= sent;

	if (ca->delta_segments >= 0 &&
	    ca->burst > sent &&
	    tcp_packets_in_flight(tp) <= tp->snd_cwnd) {
		/* Reduce the cwnd accordingly, because we didn't sent enough
		 * to cover it (we are app limited probably) */
		u32 diff = ca->burst - sent;

		if (tp->snd_cwnd >= diff)
			tp->snd_cwnd -= diff;
		else
			tp->snd_cwnd = 0;
		DBG("%u sport: %u [%s] reducing cwnd by %u, value %u\n",
		    tcp_time_stamp, ca->sport, __func__,
		    ca->burst - sent, tp->snd_cwnd);
	}
}

static void wavetcp_no_data(struct sock *sk)
{
	DBG("%u [%s]\n", tcp_time_stamp, __func__);
}

static u32 wavetcp_sndbuf_expand(struct sock *sk)
{
	return 10;
}

static struct tcp_congestion_ops wave_cong_tcp __read_mostly = {
	.init				= wavetcp_init,
	.release			= wavetcp_release,
	.ssthresh			= wavetcp_recalc_ssthresh,
/*	.cong_avoid		= wavetcp_cong_avoid, */
	.cong_control			= wavetcp_cong_control,
	.set_state			= wavetcp_state,
	.undo_cwnd			= wavetcp_undo_cwnd,
	.cwnd_event			= wavetcp_cwnd_event,
	.pkts_acked			= wavetcp_acked,
	.sndbuf_expand			= wavetcp_sndbuf_expand,
	.owner				= THIS_MODULE,
	.name				= "wave",
	.get_send_timer_exp_time	= wavetcp_get_timer,
	.send_timer_expired		= wavetcp_timer_expired,
	.no_data_to_transmit		= wavetcp_no_data,
	.segment_sent			= wavetcp_segment_sent,
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
