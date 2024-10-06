package org.firstinspires.ftc.teamcode.utilities.telemetryex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;

/**
 * Extended Telemetry for ease of processing telemetry.
 */
public class TelemetryEx {
    private final Telemetry telemetry;

    public TelemetryEx(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void print(String caption) {
        telemetry.addLine(caption);
    }

    public void print(String header, Object caption) {
        telemetry.addData(header, caption);
    }

    public void printCarousel(CarouselSelect<?> carousel) {
        StringBuilder str = new StringBuilder();
        for (int i = 0; i < carousel.getAllOptions().length; i++) {
            Object element = carousel.getAllOptions()[i];

            if (element == carousel.getSelected()) {
                str.append("◉ ").append(element.toString());
            } else {
                str.append("⭘ ").append(element.toString());
            }
        }

        telemetry.addLine(str.toString());
    }
}
