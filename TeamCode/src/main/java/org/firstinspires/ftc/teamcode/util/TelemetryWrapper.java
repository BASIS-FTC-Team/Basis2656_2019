package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TelemetryWrapper - Makes the telemetry readouts a bit more readable and manageable.
 */

public class TelemetryWrapper {
    private static Telemetry t;
    private static String[] lines;
    public static void init(Telemetry t, int nlines) {
        t.clear();
        TelemetryWrapper.t = t;
        lines = new String[nlines];
        render();
    }

    public static void render() {
        for (int i = 0; i < lines.length; i++) {
            if (lines[i] == null)
                lines[i] = "";
            t.addData("" + i, lines[i]);
        }
        t.update();
    }

    public static void setLine(int l, String message) {
        if (l < 0 || l >= lines.length) return;
        lines[l] = message;
        render();
    }

    public static void setLineNoRender(int l, String message) {
        if (l < 0 || l >= lines.length) return;
        lines[l] = message;
    }

    public static void setLines(int l) {
        t.clear();
        lines = new String[l];
        render();
    }

    public static void clear() {
        for (int i = 0; i < lines.length; i++) {
            lines[i] = "";
        }
    }

    public static void clearLines(int startLine, int endLine) {
        int sLine, eLine, tmp;
        sLine = startLine;
        eLine = endLine;
        tmp = sLine;
        if (sLine > eLine) {
            sLine = eLine;
            eLine = tmp;
        }
        if ((eLine < 0) || (sLine > lines.length-1)) {return;} //do nothing
        if (sLine < 0) { sLine = 0; }
        if (eLine > lines.length-1) { eLine = lines.length-1; }
        for (int i = sLine; i <= eLine; i++) {
            lines[i] = "";
        }
        render();
    }
}
