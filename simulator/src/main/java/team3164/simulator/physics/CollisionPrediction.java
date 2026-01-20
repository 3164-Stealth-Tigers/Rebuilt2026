package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.RobotState;

/**
 * Predictive collision avoidance for AI robots.
 * Uses trajectory prediction and weighted avoidance vectors to steer robots
 * around each other before collisions occur.
 */
public class CollisionPrediction {

    // Combined robot size for collision detection (two robots colliding)
    private static final double COLLISION_RADIUS = Constants.Robot.LENGTH_WITH_BUMPERS;

    /**
     * Result of predicting a collision between two robots.
     */
    public static class PredictionResult {
        /** Whether a collision is predicted within the lookahead time */
        public boolean willCollide = false;

        /** Time until collision (seconds), or Double.MAX_VALUE if no collision */
        public double timeToCollision = Double.MAX_VALUE;

        /** Closest approach distance between robots (meters) */
        public double closestApproachDist = Double.MAX_VALUE;

        /** Position of collision/closest approach (X) */
        public double collisionX;

        /** Position of collision/closest approach (Y) */
        public double collisionY;
    }

    /**
     * Avoidance steering vector computed from collision predictions.
     */
    public static class AvoidanceVector {
        /** Steering direction X component (normalized, -1 to 1) */
        public double steerX = 0;

        /** Steering direction Y component (normalized, -1 to 1) */
        public double steerY = 0;

        /** Urgency of avoidance (0-1, higher = more urgent) */
        public double urgency = 0;

        /** Whether any avoidance is needed */
        public boolean needsAvoidance = false;
    }

    /**
     * Predict if and when two robots will collide given their current velocities.
     * Uses quadratic trajectory math to find the time of closest approach.
     *
     * @param self The robot we're predicting for
     * @param other The other robot to check against
     * @param lookahead Maximum time horizon to check (seconds)
     * @return PredictionResult with collision details
     */
    public static PredictionResult predictCollision(RobotState self, RobotState other, double lookahead) {
        PredictionResult result = new PredictionResult();

        // Relative position (other relative to self)
        double dx = other.x - self.x;
        double dy = other.y - self.y;

        // Relative velocity (other relative to self)
        double dvx = other.vx - self.vx;
        double dvy = other.vy - self.vy;

        // Current distance
        double currentDist = Math.hypot(dx, dy);

        // Collision threshold (sum of robot radii plus buffer)
        double collisionThreshold = COLLISION_RADIUS + Constants.CollisionAvoidance.AVOIDANCE_BUFFER;

        // Already colliding
        if (currentDist < collisionThreshold) {
            result.willCollide = true;
            result.timeToCollision = 0;
            result.closestApproachDist = currentDist;
            result.collisionX = (self.x + other.x) / 2;
            result.collisionY = (self.y + other.y) / 2;
            return result;
        }

        // Solve quadratic equation for time of closest approach:
        // distance^2(t) = (dx + dvx*t)^2 + (dy + dvy*t)^2
        // d/dt(distance^2) = 2*(dx + dvx*t)*dvx + 2*(dy + dvy*t)*dvy = 0
        // => t = -(dx*dvx + dy*dvy) / (dvx^2 + dvy^2)

        double relVelSquared = dvx * dvx + dvy * dvy;

        // If relative velocity is very small, robots are moving together
        if (relVelSquared < 0.001) {
            result.closestApproachDist = currentDist;
            return result;
        }

        // Time of closest approach
        double tClosest = -(dx * dvx + dy * dvy) / relVelSquared;

        // If closest approach is in the past or beyond lookahead, no collision predicted
        if (tClosest < 0 || tClosest > lookahead) {
            // Check distance at lookahead boundary
            double futureX = dx + dvx * Math.max(0, Math.min(tClosest, lookahead));
            double futureY = dy + dvy * Math.max(0, Math.min(tClosest, lookahead));
            result.closestApproachDist = Math.hypot(futureX, futureY);
            return result;
        }

        // Calculate distance at time of closest approach
        double closestX = dx + dvx * tClosest;
        double closestY = dy + dvy * tClosest;
        double closestDist = Math.hypot(closestX, closestY);

        result.closestApproachDist = closestDist;
        result.collisionX = self.x + self.vx * tClosest;
        result.collisionY = self.y + self.vy * tClosest;

        // Will they collide?
        if (closestDist < collisionThreshold) {
            result.willCollide = true;

            // Find actual collision time (when distance = threshold)
            // Solve: (dx + dvx*t)^2 + (dy + dvy*t)^2 = threshold^2
            // a*t^2 + b*t + c = 0 where:
            // a = dvx^2 + dvy^2
            // b = 2*(dx*dvx + dy*dvy)
            // c = dx^2 + dy^2 - threshold^2

            double a = relVelSquared;
            double b = 2.0 * (dx * dvx + dy * dvy);
            double c = dx * dx + dy * dy - collisionThreshold * collisionThreshold;

            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                double sqrtDisc = Math.sqrt(discriminant);
                double t1 = (-b - sqrtDisc) / (2 * a);
                double t2 = (-b + sqrtDisc) / (2 * a);

                // Take the earliest positive collision time
                if (t1 > 0) {
                    result.timeToCollision = t1;
                } else if (t2 > 0) {
                    result.timeToCollision = t2;
                } else {
                    result.timeToCollision = tClosest;
                }
            } else {
                result.timeToCollision = tClosest;
            }
        }

        return result;
    }

    /**
     * Calculate avoidance steering vector to avoid a predicted collision.
     * Chooses a direction perpendicular to the collision vector, aligned with
     * current velocity for smooth steering.
     *
     * @param self The robot that needs to avoid
     * @param threat The robot to avoid
     * @return AvoidanceVector with steering direction and urgency
     */
    public static AvoidanceVector calculateAvoidance(RobotState self, RobotState threat) {
        AvoidanceVector avoidance = new AvoidanceVector();

        PredictionResult prediction = predictCollision(self, threat,
                Constants.CollisionAvoidance.LOOKAHEAD_TIME);

        if (!prediction.willCollide && prediction.closestApproachDist >
                Constants.CollisionAvoidance.AVOIDANCE_RAMP_START) {
            return avoidance;
        }

        avoidance.needsAvoidance = true;

        // Vector from self to threat
        double dx = threat.x - self.x;
        double dy = threat.y - self.y;
        double dist = Math.hypot(dx, dy);

        if (dist < 0.01) {
            // Extremely close, pick arbitrary direction
            avoidance.steerX = 1.0;
            avoidance.steerY = 0.0;
            avoidance.urgency = 1.0;
            return avoidance;
        }

        // Normalize threat direction
        double threatDirX = dx / dist;
        double threatDirY = dy / dist;

        // Perpendicular directions (two choices: left or right of threat)
        double perpLeftX = -threatDirY;
        double perpLeftY = threatDirX;
        double perpRightX = threatDirY;
        double perpRightY = -threatDirX;

        // Choose the perpendicular that aligns better with our current velocity
        // for smoother avoidance
        double selfSpeed = Math.hypot(self.vx, self.vy);
        double leftDot, rightDot;

        if (selfSpeed > 0.1) {
            double velDirX = self.vx / selfSpeed;
            double velDirY = self.vy / selfSpeed;
            leftDot = perpLeftX * velDirX + perpLeftY * velDirY;
            rightDot = perpRightX * velDirX + perpRightY * velDirY;
        } else {
            // Not moving much, choose based on which side is clearer
            // Prefer side away from field center for spreading out
            leftDot = perpLeftY * (self.y > Constants.Field.CENTER_Y ? 1 : -1);
            rightDot = perpRightY * (self.y > Constants.Field.CENTER_Y ? 1 : -1);
        }

        // Select the better perpendicular
        if (leftDot >= rightDot) {
            avoidance.steerX = perpLeftX;
            avoidance.steerY = perpLeftY;
        } else {
            avoidance.steerX = perpRightX;
            avoidance.steerY = perpRightY;
        }

        // Check if chosen direction would push us into an obstacle
        double testX = self.x + avoidance.steerX * 0.5;
        double testY = self.y + avoidance.steerY * 0.5;
        if (CollisionPhysics.wouldCollide(testX, testY)) {
            // Flip to other perpendicular
            avoidance.steerX = -avoidance.steerX;
            avoidance.steerY = -avoidance.steerY;
        }

        // Calculate urgency based on time to collision and distance
        double urgencyFromTime = 0;
        double urgencyFromDist = 0;

        if (prediction.willCollide) {
            // Higher urgency for sooner collisions
            urgencyFromTime = 1.0 - Math.min(1.0, prediction.timeToCollision /
                    Constants.CollisionAvoidance.LOOKAHEAD_TIME);
        }

        // Higher urgency for closer distances
        if (dist < Constants.CollisionAvoidance.AVOIDANCE_RAMP_START) {
            double rampRange = Constants.CollisionAvoidance.AVOIDANCE_RAMP_START -
                    Constants.CollisionAvoidance.AVOIDANCE_RAMP_END;
            double distIntoRamp = Constants.CollisionAvoidance.AVOIDANCE_RAMP_START - dist;
            urgencyFromDist = Math.min(1.0, distIntoRamp / rampRange);
        }

        // Combined urgency
        avoidance.urgency = Math.max(urgencyFromTime, urgencyFromDist);
        avoidance.urgency = Math.min(1.0, avoidance.urgency);

        return avoidance;
    }

    /**
     * Calculate combined avoidance vector from all nearby robots.
     * Uses weighted sum based on urgency to handle multiple threats.
     *
     * @param self The robot calculating avoidance
     * @param allRobots Array of all robots on field
     * @return Combined AvoidanceVector from all threats
     */
    public static AvoidanceVector calculateCombinedAvoidance(RobotState self, RobotState[] allRobots) {
        AvoidanceVector combined = new AvoidanceVector();

        if (allRobots == null || allRobots.length == 0) {
            return combined;
        }

        double totalWeightX = 0;
        double totalWeightY = 0;
        double maxUrgency = 0;
        int threatCount = 0;

        for (RobotState other : allRobots) {
            // Skip self
            if (other == null || other.robotId == self.robotId) {
                continue;
            }

            // Skip disabled robots
            if (!other.isEnabled) {
                continue;
            }

            // Quick distance check before expensive prediction
            double dist = Math.hypot(other.x - self.x, other.y - self.y);
            if (dist > Constants.CollisionAvoidance.AVOIDANCE_RAMP_START + 2.0) {
                continue;
            }

            AvoidanceVector avoidance = calculateAvoidance(self, other);

            if (avoidance.needsAvoidance) {
                // Weight by urgency squared for more responsive avoidance of imminent threats
                double weight = avoidance.urgency * avoidance.urgency;
                totalWeightX += avoidance.steerX * weight;
                totalWeightY += avoidance.steerY * weight;
                maxUrgency = Math.max(maxUrgency, avoidance.urgency);
                threatCount++;
            }
        }

        if (threatCount > 0) {
            combined.needsAvoidance = true;
            combined.urgency = maxUrgency;

            // Normalize the combined steering vector
            double magnitude = Math.hypot(totalWeightX, totalWeightY);
            if (magnitude > 0.01) {
                combined.steerX = totalWeightX / magnitude;
                combined.steerY = totalWeightY / magnitude;
            } else {
                // Conflicting avoidance directions cancel out - slow down instead
                combined.steerX = 0;
                combined.steerY = 0;
            }

            // Scale by max steering magnitude
            combined.steerX *= Constants.CollisionAvoidance.MAX_AVOIDANCE_STEERING;
            combined.steerY *= Constants.CollisionAvoidance.MAX_AVOIDANCE_STEERING;
        }

        return combined;
    }
}
