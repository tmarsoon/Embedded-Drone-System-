// Dataset credit: https://corgis-edu.github.io/corgis/csv/medal_of_honor/
package application;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PushbackReader;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.InvalidPathException;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/** Reads the supplied roster CSV without losing quoted names or organizations. */
public class CSVReader {
    private static final String DEFAULT_RESOURCE = "medal_of_honor.csv";

    /** Returns every valid roster record in its original, reproducible order. */
    public List<Troop> readTroopNames(String csvFile) throws IOException {
        if (csvFile == null || csvFile.isBlank()) {
            throw new IOException("A CSV filename or classpath resource is required");
        }
        try (Reader input = open(csvFile)) {
            List<List<String>> records = parse(input);
            if (records.isEmpty()) throw new IOException("CSV contains no header");
            List<String> header = records.get(0);
            int nameColumn = requiredColumn(header, "name");
            int branchColumn = requiredColumn(header, "military record.organization");
            List<Troop> troops = new ArrayList<>();
            for (int index = 1; index < records.size(); ++index) {
                List<String> record = records.get(index);
                if (record.stream().allMatch(String::isBlank)) continue;
                if (record.size() != header.size()) {
                    throw new IOException("CSV record " + (index + 1) + " has "
                            + record.size() + " fields; expected " + header.size());
                }
                String name = record.get(nameColumn).strip();
                String branch = record.get(branchColumn).strip();
                if (!name.isEmpty() && !branch.isEmpty()) troops.add(new Troop(name, branch));
            }
            return troops;
        }
    }

    /** An explicitly seeded shuffle is available to demonstrations that need it. */
    public List<Troop> readTroopNames(String csvFile, long seed) throws IOException {
        List<Troop> troops = readTroopNames(csvFile);
        Collections.shuffle(troops, new Random(seed));
        return troops;
    }

    private Reader open(String csvFile) throws IOException {
        if (csvFile.startsWith("classpath:")) {
            String name = csvFile.substring("classpath:".length());
            while (name.startsWith("/")) name = name.substring(1);
            return resource(name);
        }
        final Path path;
        try {
            path = Path.of(csvFile);
        } catch (InvalidPathException e) {
            throw new IOException("Invalid CSV path: " + csvFile, e);
        }
        if (Files.isRegularFile(path)) return Files.newBufferedReader(path, StandardCharsets.UTF_8);
        // The packaged application can launch from a different working directory.
        // An explicitly supplied missing path is never silently substituted.
        if (csvFile.equals(DEFAULT_RESOURCE)) return resource(DEFAULT_RESOURCE);
        throw new NoSuchFileException(csvFile);
    }

    private Reader resource(String name) throws IOException {
        InputStream input = CSVReader.class.getResourceAsStream("/" + name);
        if (input == null) throw new NoSuchFileException("classpath:" + name);
        return new InputStreamReader(input, StandardCharsets.UTF_8);
    }

    private int requiredColumn(List<String> header, String required) throws IOException {
        int result = -1;
        for (int index = 0; index < header.size(); ++index) {
            if (header.get(index).strip().toLowerCase(Locale.ROOT).equals(required)) {
                if (result >= 0) throw new IOException("Duplicate CSV header: " + required);
                result = index;
            }
        }
        if (result < 0) throw new IOException("Missing CSV header: " + required);
        return result;
    }

    /** RFC 4180 fields, including embedded newlines and doubled quote escapes. */
    private List<List<String>> parse(Reader source) throws IOException {
        PushbackReader input = new PushbackReader(source, 1);
        List<List<String>> records = new ArrayList<>();
        List<String> record = new ArrayList<>();
        StringBuilder field = new StringBuilder();
        boolean quoted = false, afterQuote = false, started = false;
        int first = input.read();
        if (first >= 0 && first != '\uFEFF') input.unread(first);
        for (int value; (value = input.read()) >= 0;) {
            char character = (char) value;
            if (quoted) {
                if (character == '"') {
                    int next = input.read();
                    if (next == '"') field.append('"');
                    else {
                        quoted = false;
                        afterQuote = true;
                        if (next >= 0) input.unread(next);
                    }
                } else field.append(character);
                continue;
            }
            if (character == ',') {
                record.add(field.toString());
                field.setLength(0);
                afterQuote = false;
                started = true;
            } else if (character == '\r' || character == '\n') {
                record.add(field.toString());
                records.add(record);
                record = new ArrayList<>();
                field.setLength(0);
                afterQuote = false;
                started = false;
                if (character == '\r') {
                    int next = input.read();
                    if (next >= 0 && next != '\n') input.unread(next);
                }
            } else if (afterQuote) {
                // Permit insignificant whitespace after a closing quote; all
                // other text would make field boundaries ambiguous.
                if (character != ' ' && character != '\t') {
                    throw new IOException("Unexpected text after quoted field in CSV record "
                            + (records.size() + 1));
                }
            } else if (character == '"') {
                if (field.length() != 0) {
                    throw new IOException("Unexpected quote in CSV record " + (records.size() + 1));
                }
                quoted = true;
                started = true;
            } else {
                field.append(character);
                started = true;
            }
        }
        if (quoted) throw new IOException("Unterminated quoted field in CSV record " + (records.size() + 1));
        if (started || afterQuote || !record.isEmpty()) {
            record.add(field.toString());
            records.add(record);
        }
        return records;
    }
}
