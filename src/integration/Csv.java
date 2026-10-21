package integration;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Strict single-record RFC4180 parsing; native telemetry fields contain no embedded newlines. */
public final class Csv {
    private Csv() {}
    public static List<String> parseLine(String line) throws IOException {
        List<String> cells = new ArrayList<>();
        StringBuilder cell = new StringBuilder();
        boolean quoted = false, closed = false;
        for (int i = 0; i < line.length(); ++i) {
            char c = line.charAt(i);
            if (quoted) {
                if (c == '"') {
                    if (i + 1 < line.length() && line.charAt(i + 1) == '"') {
                        cell.append('"'); ++i;
                    } else { quoted = false; closed = true; }
                } else { cell.append(c); }
            } else if (c == ',') {
                cells.add(cell.toString()); cell.setLength(0); closed = false;
            } else if (c == '"' && cell.isEmpty() && !closed) {
                quoted = true;
            } else if (closed || c == '"' || c == '\r' || c == '\n') {
                throw new IOException("Malformed CSV record");
            } else { cell.append(c); }
        }
        if (quoted) throw new IOException("Unterminated CSV field");
        cells.add(cell.toString());
        return List.copyOf(cells);
    }
}
