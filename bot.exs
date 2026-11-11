# An Elixir robot on the core binding supplied by the arena.
# Precise contacts lead shots; remembered passive cues guide the search.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
defmodule Bot do
  alias Clunkyard, as: C
  def on_start(robot), do: C.match_is_3d(robot) and C.arena_gravity(robot).z == 0
  defp clamp(value), do: C.angles_clamp(value, -1, 1)
  defp steer(towards, current), do: clamp(C.angles_angle_diff(towards, current) / 45)
  def on_turn(r, free) do
    pos = C.me_position(r)
    heading = C.me_heading(r)
    pitch = C.me_pitch(r)
    C.drive_forces(r, %{brake: 0.2})
    target = C.sensors_nearest(r)
    cue = C.tracker_cue(r)
    cond do
      target != :undefined ->
        throttle = cond do target.range > 350 -> 0.7; target.range < 200 -> -0.4; true -> 0.2 end
        C.drive_set(r, %{throttle: throttle, turn: steer(target.bearing, heading), pitch_turn: steer(target.elevation, pitch)})
        C.scanner_set(r, %{heading: target.bearing, arc: 20, pitch: target.elevation})
        power = C.gun_power_for_flight_time(r, target.range, 1)
        aim = C.gun_lead(r, target, power)
        {h, p} = if aim == :undefined, do: {target.bearing, target.elevation}, else: {aim.heading, aim.pitch}
        C.gun_aim(r, %{heading: h, pitch: p})
        now = C.gun_aim_now(r)
        if abs(C.angles_angle_diff(h, now.heading)) < 3 and abs(p - now.pitch) < 3 and C.gun_can_fire(r, power, 10), do: C.gun_fire(r, %{power: power})
        C.status_set(r, %{text: "TRACK", value: target.range})
      cue != :undefined ->
        bearing = C.angles_bearing_to(pos, cue.position)
        distance = C.angles_distance(pos, cue.position)
        elevation = if free, do: C.angles_elevation_to(pos, cue.position), else: 0
        error = min(cue.bearing_error, 30)
        tick = C.clock_tick(r)
        across = (rem(tick, 5) - 2) / 2 * error
        up = if free, do: (rem(div(tick, 5), 3) - 1) * error / 2, else: 0
        C.drive_set(r, %{throttle: 0.7, turn: steer(bearing, heading), pitch_turn: steer(elevation, pitch)})
        C.scanner_set(r, %{heading: bearing + across, arc: C.scanner_arc_for_reach(r, distance + 100), pitch: elevation + up})
        C.status_set(r, %{text: "CUE", value: distance})
      true ->
        up = if free, do: (rem(div(C.clock_tick(r), 10), 3) - 1) * 35, else: 0
        C.drive_set(r, %{throttle: 0.5, turn: 0.33, pitch_turn: steer(0, pitch)})
        C.scanner_set(r, %{heading: C.angles_normalize(C.scanner_last(r).heading + 36), arc: 40, pitch: up})
        C.status_set(r, %{text: "SEARCH"})
    end
    if C.battery_charge(r) < C.battery_capacity(r) / 4, do: C.drive_set(r, %{throttle: C.drive_get(r, :throttle) * 0.5})
    storm = C.nav_storm(r)
    if storm != :undefined and C.nav_storm_margin(r, pos) > -60 do
      away = C.angles_bearing_to(storm.center, pos)
      C.drive_set(r, %{throttle: 0.8, turn: clamp(C.angles_angle_diff(away, heading) / 30)})
    end
    zone = C.nav_zone(r)
    if zone != :undefined and C.nav_zone_margin(r, pos) < 60 do
      climb = if zone.shape == "sphere", do: C.angles_elevation_to(pos, zone.center), else: 0
      C.drive_set(r, %{throttle: 0.8, turn: clamp(C.angles_angle_diff(C.angles_bearing_to(pos, zone.center), heading) / 30), pitch_turn: clamp((climb - pitch) / 30)})
    end
    # The arena's rate calibration, xorshift-sqrt v1; safe to delete.
    work = C.raw_number(r, "state.calibration.work")
    if is_number(work) and work > 0, do: C.status_set(r, %{text: "WORK", value: calibration_work(trunc(work), C.clock_tick(r))})
    free
  end
  defp calibration_work(work, tick) do
    import Bitwise
    mask = 0xFFFFFFFFFFFFFFFF
    {x, sum} = Enum.reduce(1..work, {bxor(88172645463325252, tick), 0.0}, fn _, {x, sum} ->
      x = bxor(x, band(x <<< 13, mask)); x = bxor(x, x >>> 7); x = bxor(x, band(x <<< 17, mask))
      {x, sum + :math.sqrt(x >>> 11)}
    end)
    rem(floor(sum) + (x >>> 40), 1000000)
  end
end
Clunkyard.run(Bot)
