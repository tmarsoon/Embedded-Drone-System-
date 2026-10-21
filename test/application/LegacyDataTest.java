package application;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class LegacyDataTest {
    @TempDir Path temporary;

    private Path csv(String name, String contents) throws IOException {
        Path path = temporary.resolve(name);
        Files.writeString(path, contents, StandardCharsets.UTF_8);
        return path;
    }

    private List<String> identities(List<Troop> troops) {
        return troops.stream().map(t -> t.getName() + "|" + t.getBranch()).toList();
    }

    @Test void quotedNamesEscapedQuotesMultilineUtf8BomAndReorderedHeaders() throws IOException {
        Path source = csv("quoted.csv", "\uFEFFmilitary record.rank,name,military record.organization\r\n"
                + "Sergeant,\"Doe, Jane \"\"JJ\"\"\nResearcher\",U.S. Army\r\n"
                + "Captain,\"García, José\",U.S. Navy\r\n");
        List<Troop> troops = new CSVReader().readTroopNames(source.toString());
        assertEquals(2, troops.size());
        assertEquals("Doe, Jane \"JJ\"\nResearcher", troops.get(0).getName());
        assertEquals("U.S. Army", troops.get(0).getBranch());
        assertEquals("García, José", troops.get(1).getName());
        assertEquals("U.S. Navy", troops.get(1).getBranch());
    }

    @Test void originalDatasetRetainsAllNamesAndOrganizationCategories() throws IOException {
        List<Troop> troops = new CSVReader().readTroopNames("medal_of_honor.csv");
        assertEquals(3473, troops.size());
        assertEquals("Sagelhurst, John C.", troops.get(0).getName());
        assertEquals("U.S. Army", troops.get(0).getBranch());
        Map<String, Long> counts = troops.stream().map(Troop::getBranch)
                .collect(Collectors.groupingBy(Function.identity(), Collectors.counting()));
        assertEquals(Map.of("U.S. Army", 2426L, "U.S. Navy", 734L,
                "U.S. Marine Corps", 295L, "U.S. Air Force", 17L,
                "U.S. Coast Guard", 1L), counts);
    }

    @Test void packagedDatasetLoadsWithoutFilesystemLocation() throws IOException {
        List<Troop> troops = new CSVReader().readTroopNames("classpath:/medal_of_honor.csv");
        assertEquals(3473, troops.size());
        assertEquals("Sagelhurst, John C.", troops.get(0).getName());
    }

    @Test void defaultAndExplicitSeededOrdersAreReproducible() throws IOException {
        CSVReader reader = new CSVReader();
        List<String> first = identities(reader.readTroopNames("medal_of_honor.csv"));
        assertEquals(first, identities(reader.readTroopNames("medal_of_honor.csv")));
        List<String> shuffled = identities(reader.readTroopNames("medal_of_honor.csv", 42));
        assertEquals(shuffled, identities(reader.readTroopNames("medal_of_honor.csv", 42)));
        assertNotEquals(first, shuffled);
        assertEquals(first.stream().sorted().toList(), shuffled.stream().sorted().toList());
    }

    @Test void everyValidRecordIsReturnedWithoutLegacy3500RowCutoff() throws IOException {
        StringBuilder input = new StringBuilder("name,military record.organization,military record.rank\n");
        for (int i = 0; i < 3505; ++i) input.append("Person ").append(i).append(",U.S. Army,Sergeant\n");
        assertEquals(3505, new CSVReader().readTroopNames(csv("large.csv", input.toString()).toString()).size());
    }

    @Test void blankRecordsAndBlankRequiredCellsAreSkipped() throws IOException {
        Path source = csv("blanks.csv", "name,military record.organization,military record.rank\n"
                + "\n,,\n, U.S. Army,Sergeant\nValid,,Sergeant\nValid,U.S. Army,\n");
        List<Troop> troops = new CSVReader().readTroopNames(source.toString());
        assertEquals(1, troops.size());
        assertEquals("Valid", troops.get(0).getName());
    }

    @Test void malformedCsvAndHeadersAreRejectedInsteadOfMisassigned() throws IOException {
        CSVReader reader = new CSVReader();
        for (String contents : List.of("", "name,rank\nPerson,Sergeant\n",
                "name,name,military record.organization\nA,B,U.S. Army\n",
                "name,military record.organization\n\"Unclosed,U.S. Army\n",
                "name,military record.organization\n\"Name\"junk,U.S. Army\n",
                "name,military record.organization\nA\"B,U.S. Army\n",
                "name,military record.organization\nA,U.S. Army,unexpected\n")) {
            Path source = csv("malformed.csv", contents);
            assertThrows(IOException.class, () -> reader.readTroopNames(source.toString()), contents);
        }
        assertThrows(IOException.class, () -> reader.readTroopNames(temporary.resolve("missing.csv").toString()));
        assertThrows(IOException.class, () -> reader.readTroopNames("classpath:missing-roster.csv"));
        assertThrows(IOException.class, () -> reader.readTroopNames(" "));
    }

    @Test void loggerAppendsNewOutputAndPreservesHistoricalLogs() throws IOException {
        Path historical = temporary.resolve("simulation_log.txt");
        Files.writeString(historical, "historical upload\n", StandardCharsets.UTF_8);
        Path active = temporary.resolve("runs/logs/simulation.log");
        Files.createDirectories(active.getParent());
        Files.writeString(active, "existing run\n", StandardCharsets.UTF_8);
        String oldProjectRoot = System.getProperty("drone.project.root");
        String oldRoot = System.getProperty("ustranscomm.root");
        String oldDirectory = System.getProperty("ustranscomm.logDir");
        try {
            System.setProperty("drone.project.root", temporary.toString());
            System.setProperty("ustranscomm.root", temporary.toString());
            System.clearProperty("ustranscomm.logDir");
            Logger.log("new event");
            Logger.log("second event");
            assertEquals("historical upload\n", Files.readString(historical, StandardCharsets.UTF_8));
            String logged = Files.readString(active, StandardCharsets.UTF_8);
            assertTrue(logged.startsWith("existing run\n"), logged);
            assertTrue(logged.contains("new event"), logged);
            assertTrue(logged.contains("second event"), logged);
        } finally {
            if (oldProjectRoot == null) System.clearProperty("drone.project.root");
            else System.setProperty("drone.project.root", oldProjectRoot);
            if (oldRoot == null) System.clearProperty("ustranscomm.root");
            else System.setProperty("ustranscomm.root", oldRoot);
            if (oldDirectory == null) System.clearProperty("ustranscomm.logDir");
            else System.setProperty("ustranscomm.logDir", oldDirectory);
        }
    }
}
