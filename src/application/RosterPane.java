package application;

import javafx.beans.property.ReadOnlyStringWrapper;
import javafx.collections.FXCollections;
import javafx.collections.transformation.FilteredList;
import javafx.geometry.Insets;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;

import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.TreeSet;

/** Historical roster from the uploaded template; independent of synthetic flight demand. */
public final class RosterPane extends BorderPane {
    public RosterPane(Path projectRoot) {
        setPadding(new Insets(18));
        Label heading=new Label("Original historical roster");
        heading.setStyle("-fx-font-size:21px;-fx-font-weight:bold;");
        Label description=new Label("The supplied Medal of Honor CSV is preserved as historical context. Names do not represent current personnel or simulated delivery assignments.");
        description.setWrapText(true);
        Label count=new Label();
        TableView<Troop> table=new TableView<>();
        table.setColumnResizePolicy(TableView.CONSTRAINED_RESIZE_POLICY_FLEX_LAST_COLUMN);
        TableColumn<Troop,String> names=new TableColumn<>("Name");
        names.setCellValueFactory(cell->new ReadOnlyStringWrapper(cell.getValue().getName()));
        TableColumn<Troop,String> branches=new TableColumn<>("Branch / organization");
        branches.setCellValueFactory(cell->new ReadOnlyStringWrapper(cell.getValue().getBranch()));
        table.getColumns().addAll(names,branches);
        TextField search=new TextField();search.setPromptText("Search historical names");
        ComboBox<String> branch=new ComboBox<>();branch.getItems().add("All branches");branch.getSelectionModel().selectFirst();
        HBox filters=new HBox(10,search,branch,count);HBox.setHgrow(search,Priority.ALWAYS);
        VBox top=new VBox(9,heading,description,filters);top.setPadding(new Insets(0,0,14,0));setTop(top);setCenter(table);
        try {
            List<Troop> troops=new CSVReader().readTroopNames(projectRoot.resolve("medal_of_honor.csv").toString());
            TreeSet<String> organizations=new TreeSet<>();troops.forEach(t->organizations.add(t.getBranch()));branch.getItems().addAll(organizations);
            FilteredList<Troop> filtered=new FilteredList<>(FXCollections.observableArrayList(troops));
            Runnable refresh=()->{
                String term=search.getText().strip().toLowerCase(Locale.ROOT);
                String selected=branch.getValue();
                filtered.setPredicate(t->t.getName().toLowerCase(Locale.ROOT).contains(term)
                    && ("All branches".equals(selected)||t.getBranch().equals(selected)));
                count.setText(filtered.size()+" / "+troops.size()+" records");
            };
            search.textProperty().addListener((o,a,b)->refresh.run());branch.valueProperty().addListener((o,a,b)->refresh.run());
            table.setItems(filtered);refresh.run();
        } catch(Exception error) {
            count.setText("Roster unavailable");
            table.setPlaceholder(new Label("The flight simulator is still available. CSV load error: "+error.getMessage()));
        }
    }
}
