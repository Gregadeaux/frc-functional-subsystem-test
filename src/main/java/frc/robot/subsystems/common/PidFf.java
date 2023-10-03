package frc.robot.subsystems.common;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;

public class PidFf {
    public static record PidFfGoal(double goalValue) {}
    public static interface ToPidFfGoal {
        public PidFfGoal toPidFfGoal();
    }

    public static record PidFfOutput(Optional<Double> effortVoltage) {}
    public interface PidFfOutputFactory<T> {
        public T fromPidFfOutput(PidFfOutput output);
    }

    public static record ArmPidFfInput(
        double pidMeasurement,
        double positionRadians,
        double velocityRadians,
        double accelRadians
    ) {
        public static class Builder {
            private double pidMeasurement;
            private double positionRadians;
            private double velocityRadians;
            private double accelRadians = 0;

            public static Builder withFeedbackMeasurement(double measurement) {
                final Builder builder = new Builder();
                builder.pidMeasurement = measurement;
                return builder;
            }

            public static Builder withFeedfowardPositionRadians(double positionRadians) {
                final Builder builder = new Builder();
                builder.positionRadians = positionRadians;
                return builder;
            }

            public static Builder withFeedfowardVelocityRadians(double velocityRadians) {
                final Builder builder = new Builder();
                builder.velocityRadians = velocityRadians;
                return builder;
            }

            public static Builder withFeedfowardAccelRadians(double accelRadians) {
                final Builder builder = new Builder();
                builder.accelRadians = accelRadians;
                return builder;
            }

            public Builder plusFeedbackMeasurement(double measurement) {
                this.pidMeasurement = measurement;
                return this;
            }

            public Builder plusFeedforwardPositionRadians(double positionRadians) {
                this.positionRadians = positionRadians;
                return this;
            }

            public Builder plusFeedforwardVelocityRadians(double velocityRadians) {
                this.velocityRadians = velocityRadians;
                return this;
            }

            public Builder plusFeedforwardAccelRadians(double accelRadians) {
                this.accelRadians = accelRadians;
                return this;
            }

            public ArmPidFfInput build() {
                return new ArmPidFfInput(pidMeasurement, positionRadians, velocityRadians, accelRadians);
            }
        }
    }
    public static interface ToArmPidFfInput {
        public ArmPidFfInput toArmPidFfInput();
    }

    public static record ArmPidFfMetadata(ProfiledPIDController pidController) {}

    public static class ArmPidFfBehavior<IN extends ToArmPidFfInput, OUT, GOAL extends ToPidFfGoal> extends Behavior<IN, OUT, GOAL, ArmPidFfMetadata> {
        private final PidFfOutputFactory<OUT> out;
        private final ProfiledPIDController controller;
        private final ArmFeedforward ff;

        public ArmPidFfBehavior(ProfiledPIDController controller, ArmFeedforward ff, PidFfOutputFactory<OUT> out) {
            this.out = out;
            this.controller = controller;
            this.ff = ff;

            this.setState(new ArmPidFfMetadata(this.controller));
        }

        public ArmPidFfMetadata init() {
            return new ArmPidFfMetadata(this.controller);
        }

        @Override
        public OUT periodic(final IN inputs, final GOAL goal, final ArmPidFfMetadata state) {
            return this.out.fromPidFfOutput(periodic(inputs.toArmPidFfInput(), goal.toPidFfGoal()));
        }

        private PidFfOutput periodic(ArmPidFfInput inputs, PidFfGoal goal) {
            if (DriverStation.isEnabled()) {
                Optional<Double> voltage = Optional.of(inputs)
                    .map( i -> new double[] { 
                        controller.calculate(i.pidMeasurement(), goal.goalValue()),
                        ff.calculate(i.positionRadians(), i.velocityRadians(), i.accelRadians())
                    }).map(tuple -> tuple[0] + tuple[1]);
                
                setState(new ArmPidFfMetadata(controller));
                return new PidFfOutput(voltage);
            }
            return new PidFfOutput(Optional.empty());
        }
    }
}
