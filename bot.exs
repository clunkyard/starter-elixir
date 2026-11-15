# An Elixir robot on the core binding supplied by the arena.
# Contacts narrow the beam and lead shots from a line fitted through recent
# sightings; remembered passive cues guide the search.
# Bearing is earned: a scan contact's bearing is off by up to its
# bearing_error (degrees, a fixed share of the arc), so a wide beam finds a
# robot and a narrow one aims at it. Contacts within 120 units are exact.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
#   from_mount(heading pitch) mount_toward(point)
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   history(slot) last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) to_chassis(heading pitch attitude)
#   from_chassis(heading pitch attitude) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
defmodule Bot do
  alias Clunkyard, as: C
  @search_arc 40   # degrees: sweeps the circle in half a second
  @hold_width 60   # units across at the target: it stays in the beam from tick to tick
  @min_arc 6       # degrees: the narrowest the beam holds
  @window 12       # ticks of sightings the aim averages
  @sightings 8     # narrow sightings in the window before a shot
  @look_every 40   # ticks between looks around the target, for a nearer robot
  @reacquire 10    # ticks to look again where a lost target was
  def on_start(robot) do
    free = C.match_is_3d(robot) and C.arena_gravity(robot).z == 0
    %{free: free, dodges: 0, dodge_end: 0, slot: 0}
  end
  defp clamp(value), do: C.angles_clamp(value, -1, 1)
  defp steer(towards, current), do: clamp(C.angles_angle_diff(towards, current) / 45)
  # A straight line through the narrowest of the last @window ticks' sightings:
  # where the robot is now, how fast it moves, and how many sightings it rests
  # on. Each sighting is off by its bearing_error; the line averages that out.
  defp fit(r, slot, tick) do
    recent = Enum.filter(C.tracker_history(r, slot), &(&1.tick > tick - @window))
    best = if recent == [], do: 0, else: recent |> Enum.map(& &1.bearing_error) |> Enum.min()
    seen = Enum.filter(recent, &(&1.bearing_error <= 1.5 * best))
    n = length(seen)
    if length(recent) < 2 or n < 2 do
      nil
    else
      t0 = Enum.sum(Enum.map(seen, & &1.tick)) / n
      spread = Enum.sum(Enum.map(seen, &((&1.tick - t0) * (&1.tick - t0))))
      [{x, vx}, {y, vy}, {z, vz}] =
        for axis <- [:x, :y, :z] do
          mean = Enum.sum(Enum.map(seen, &Map.fetch!(&1.position, axis))) / n
          slope = Enum.sum(Enum.map(seen, &((&1.tick - t0) * (Map.fetch!(&1.position, axis) - mean)))) / spread
          {mean + slope * (tick - t0), slope * C.match_tick_rate(r)}
        end
      {%{x: x, y: y, z: z}, %{x: vx, y: vy, z: vz}, n}
    end
  end
  def on_turn(r, m) do
    free = m.free
    pos = C.me_position(r)
    heading = C.me_heading(r)
    pitch = C.me_pitch(r)
    tick = C.clock_tick(r)
    C.drive_forces(r, %{brake: 0.2})
    target = C.sensors_nearest(r)
    cue = C.tracker_cue(r)
    last_seen = if m.slot == 0, do: :undefined, else: C.tracker_last_seen(r, m.slot)
    lost = last_seen != :undefined and tick - last_seen <= @reacquire
    m = if target != :undefined, do: %{m | slot: target.slot}, else: m
    cond do
      target != :undefined ->
        throttle = cond do target.range > 350 -> 0.7; target.range < 200 -> -0.4; true -> 0.2 end
        C.drive_set(r, %{throttle: throttle, turn: steer(target.bearing, heading), pitch_turn: steer(target.elevation, pitch)})
        # Halve the beam each tick until it is @hold_width across at the target:
        # narrow enough for a precise bearing, wide enough to keep it in.
        hold = max(@min_arc, 2 * :math.atan2(@hold_width / 2, target.range) * 180 / :math.pi())
        # In a crowd a narrow beam sees one robot. Every @look_every ticks,
        # widen it for a tick: a nearer robot it finds becomes the target.
        arc = if rem(tick, @look_every) == 0, do: @search_arc, else: max(hold, min(@search_arc, C.scanner_last(r).arc / 2))
        C.scanner_set(r, %{heading: target.bearing, arc: arc, pitch: target.elevation})
        power = C.gun_power_for_flight_time(r, target.range, 1)
        # An exact contact: gun_lead is the intercept if the target holds
        # course. A noisy one: lead the fitted line, and fire only once it
        # rests on enough sightings.
        {aim, ready} =
          if target.bearing_error > 0 do
            case fit(r, target.slot, tick) do
              nil -> {:undefined, false}
              {position, velocity, count} -> {C.gun_lead_point(r, position, velocity, power), count >= @sightings}
            end
          else
            {C.gun_lead(r, target, power), true}
          end
        {h, p} = if aim == :undefined, do: {target.bearing, target.elevation}, else: {aim.heading, aim.pitch}
        C.gun_aim(r, %{heading: h, pitch: p})
        now = C.gun_aim_now(r)
        if ready and abs(C.angles_angle_diff(h, now.heading)) < 3 and abs(p - now.pitch) < 3 and C.gun_can_fire(r, power, 10), do: C.gun_fire(r, %{power: power})
        C.status_set(r, %{text: if(ready, do: "TRACK", else: "NARROW"), value: target.range})
      lost ->
        # Lost it: look again where it was last seen, widening the beam, before searching.
        where = C.tracker_position(r, m.slot)
        bearing = C.angles_bearing_to(pos, where)
        elevation = if free, do: C.angles_elevation_to(pos, where), else: 0
        C.drive_set(r, %{throttle: 0.5, turn: steer(bearing, heading), pitch_turn: steer(elevation, pitch)})
        C.scanner_set(r, %{heading: bearing, arc: min(@search_arc, C.scanner_last(r).arc * 2), pitch: elevation})
        C.status_set(r, %{text: "REACQUIRE"})
      cue != :undefined ->
        bearing = C.angles_bearing_to(pos, cue.position)
        distance = C.angles_distance(pos, cue.position)
        elevation = if free, do: C.angles_elevation_to(pos, cue.position), else: 0
        error = min(cue.bearing_error, 30)
        across = (rem(tick, 5) - 2) / 2 * error
        up = if free, do: (rem(div(tick, 5), 3) - 1) * error / 2, else: 0
        C.drive_set(r, %{throttle: 0.7, turn: steer(bearing, heading), pitch_turn: steer(elevation, pitch)})
        C.scanner_set(r, %{heading: bearing + across, arc: C.scanner_arc_for_reach(r, distance + 100), pitch: elevation + up})
        C.status_set(r, %{text: "CUE", value: distance})
      true ->
        up = if free, do: (rem(div(tick, 10), 3) - 1) * 35, else: 0
        C.drive_set(r, %{throttle: 0.5, turn: 0.33, pitch_turn: steer(0, pitch)})
        C.scanner_set(r, %{heading: C.angles_normalize(C.scanner_last(r).heading + 36), arc: @search_arc, pitch: up})
        C.status_set(r, %{text: "SEARCH"})
    end
    # A simple dodge: vary it, or predict the shooter.
    m =
      if tick >= m.dodge_end and Enum.any?(C.sensors_cues(r), &(&1.source == "muzzle")),
        do: %{m | dodges: m.dodges + 1, dodge_end: tick + 10},
        else: m
    if tick < m.dodge_end do
      # Off the line of fire: left, then right next time (turn is positive left, strafe right).
      left = rem(m.dodges, 2) == 1
      if C.match_control(r) == "forces" do
        C.drive_set(r, %{throttle: 0.5})
        C.drive_forces(r, %{strafe: if(left, do: -1, else: 1)})
      else
        C.drive_set(r, %{throttle: 1, turn: if(left, do: 1, else: -1)})
      end
      C.status_set(r, %{text: if(left, do: "dodge:left", else: "dodge:right"), value: m.dodges})
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
    m
  end

end
Clunkyard.run(Bot)
