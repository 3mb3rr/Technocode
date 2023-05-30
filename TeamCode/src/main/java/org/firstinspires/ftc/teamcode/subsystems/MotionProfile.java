package org.firstinspires.ftc.teamcode.subsystems;

public class MotionProfile {
    public MotionProfile() {

    }
    public double desiredPosition(double max_acceleration, double max_velocity, double distance, double current_dt) {


        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * acceleration_dt*acceleration_dt;

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (current_dt > entire_dt)
            return distance;

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * current_dt*current_dt;

        // if we're cruising
  else if (current_dt < deacceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
            double cruise_current_dt = current_dt - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt*acceleration_dt;
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = current_dt - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * deacceleration_time * deacceleration_time;
        }
    }
}
