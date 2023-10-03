package frc.robot.subsystems.common;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;

public record ClampPlug<OUT>( 
    double min, 
    double max,
    Function<OUT, Optional<Double>> supplier, 
    BiFunction<Optional<Double>, OUT, OUT> consumer
) implements Plug<OUT> {

    @Override
    public OUT effect(OUT out) {
        Optional<Double> supply = supplier.apply(out)
            .map(val -> MathUtil.clamp(val, min, max))
            .or(Optional::empty);

        return consumer.apply(supply, out);
    }
    
}
