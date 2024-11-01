package org.firstinspires.ftc.teamcode.utilities.telemetryex;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;

/**
 * Extended Telemetry for ease of processing telemetry.
 */
public class TelemetryEx {
    private final MultipleTelemetry telemetry;

    public TelemetryEx(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry);
    }

    /**
     * Prints a line to telemetry
     *
     * @param caption What to print.
     */
    public void print(String caption) {
        telemetry.addLine(caption);
    }

    /**
     * Prints a line to telemetry in the format: "[Header]: [Caption]"
     *
     * @param header  What to print as the header.
     * @param caption What to print as the caption.
     */
    public void print(String header, Object caption) {
        telemetry.addData(header, caption);
    }

    /**
     * Prints a line to telemetry in the format: "◉ option1  ⭘ option2  ⭘ option3" given a CarouselSelect
     *
     * @param carousel A CarouselSelect to display the information of.
     */
    public void printCarousel(CarouselSelect<?> carousel) {
        StringBuilder str = new StringBuilder();
        for (int i = 0; i < carousel.getAllOptions().length; i++) {
            Object element = carousel.getAllOptions()[i];

            if (element == carousel.getSelected()) {
                str.append("◉ ").append(element.toString()).append("  ");
            } else {
                str.append("⭘ ").append(element.toString()).append("  ");
            }
        }

        telemetry.addLine(str.toString());
    }
}
