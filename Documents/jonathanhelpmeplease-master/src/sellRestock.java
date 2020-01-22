import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import java.util.ArrayList;

public class sellRestock
{
    public static void transaction(int modItemIn, ArrayList<Product> modArrayIn)
    {
        Stage window = new Stage();

        // Name
        Label nameTxt = new Label("Product Name: " + modArrayIn.get(modItemIn).getName());
        // Stock
        Label stockTxt = new Label("Currently on stock: " + modArrayIn.get(modItemIn).getStockLevel());
        // Selling price
        Label currentPrice = new Label("Price: $ " + modArrayIn.get(modItemIn).getPrice());
        // Radio button group
        final ToggleGroup group = new ToggleGroup();

        // Radio buttons
        RadioButton sell = new RadioButton("Sell");
        sell.setToggleGroup(group);
        sell.setSelected(true);

        RadioButton reStockOpt = new RadioButton("Restock");
        reStockOpt.setToggleGroup(group);

        // Quantity Input and submit Button
        TextField quantity = new TextField();
        quantity.setPromptText("Quantity");
        quantity.setMaxWidth(80);

        Button submit = new Button("Submit");
        submit.setOnAction(e -> {
            try {
                int quantityInt = Integer.parseInt(quantity.getText());

                if(quantityInt > 0) {
                    if (sell.isSelected())
                        if(quantityInt <= modArrayIn.get(modItemIn).getStockLevel())
                        {
                            modArrayIn.get(modItemIn).sell(quantityInt);

                            // Display current stock
                            Alert alert = new Alert(Alert.AlertType.INFORMATION);
                            alert.setTitle("Info");
                            alert.setHeaderText("Transaction successful!");
                            alert.setContentText("Current Stock: " + modArrayIn.get(modItemIn).getStockLevel()
                                    + "\nSold: " + quantityInt);

                            alert.showAndWait();
                            window.close();
                        }
                        else
                        {
                            Alert alert = new Alert(Alert.AlertType.ERROR);
                            alert.setTitle("Error");
                            alert.setHeaderText("Transaction Failed");
                            alert.setContentText("Value must be smaller or equal to the stock !");

                            alert.showAndWait();
                        }
                    else
                    {
                        modArrayIn.get(modItemIn).reStock(quantityInt);

                        // Display current stock
                        Alert alert = new Alert(Alert.AlertType.INFORMATION);
                        alert.setTitle("Info");
                        alert.setHeaderText("Transaction successful!");
                        alert.setContentText("Current Stock: " + modArrayIn.get(modItemIn).getStockLevel()
                                + "\nAdded: " + quantityInt);

                        alert.showAndWait();
                        window.close();
                    }
                } else {
                    Alert alert = new Alert(Alert.AlertType.ERROR);
                    alert.setTitle("Error");
                    alert.setHeaderText("Transaction Failed");
                    alert.setContentText("Value must be positive !");

                    alert.showAndWait();
                }
            }catch(NumberFormatException y) {
                Alert alert = new Alert(Alert.AlertType.ERROR);
                alert.setTitle("Error");
                alert.setHeaderText("Transaction Failed");
                alert.setContentText("Quantity field only allows for integer values");

                alert.showAndWait();
            }
        });

        // HBox for quantity and submit
        HBox transaction = new HBox();
        transaction.setSpacing(10);
        transaction.setPadding(new Insets(10));
        transaction.getChildren().addAll(quantity, submit);

        // Put radio buttons next to each other
        HBox hBox = new HBox();
        hBox.setSpacing(10);
        hBox.setPadding(new Insets(10));
        hBox.getChildren().addAll(sell,reStockOpt);

        VBox vBox = new VBox();
        vBox.setSpacing(10);
        vBox.setPadding(new Insets(10));
        vBox.getChildren().addAll(nameTxt,stockTxt, currentPrice, hBox, transaction);

        Scene scene = new Scene(vBox, 300,250);
        window.setTitle("Sell/Restock Chaosclothingllc");
        window.setScene(scene);
        window.showAndWait();
    }
}
