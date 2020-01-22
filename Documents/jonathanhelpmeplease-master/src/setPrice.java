import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import java.util.ArrayList;

public class setPrice
{
    public static void change(int modItemIn, ArrayList<Product> modArrayIn)
    {
        Stage window = new Stage();

        // Name
        Label nameTxt = new Label("Product Name: " + modArrayIn.get(modItemIn).getName());
        // Selling price
        Label currentPrice = new Label("Price: $ " + modArrayIn.get(modItemIn).getPrice());

        // Quantity Input and submit Button
        TextField price = new TextField();
        price.setPromptText("Price");
        price.setMaxWidth(80);

        Button submit = new Button("Submit");
        submit.setOnAction(e -> {
            try {
                double priceDbl = Double.parseDouble(price.getText());

                if(priceDbl > 0) {
                    modArrayIn.get(modItemIn).setPrice(priceDbl);


                    // Display current price
                    Alert alert = new Alert(Alert.AlertType.INFORMATION);
                    alert.setTitle("Info");
                    alert.setHeaderText("Attempt successful!");
                    alert.setContentText("Current Stock: " + modArrayIn.get(modItemIn).getPrice());

                    alert.showAndWait();
                    window.close();
                }

            }catch(NumberFormatException k){
                Alert alert = new Alert(Alert.AlertType.ERROR);
                alert.setTitle("Error");
                alert.setHeaderText("Attempt Failed");
                alert.setContentText("Value must be a number !");

                alert.showAndWait();
            }
        });

        // HBox for price and submit
        HBox hBox = new HBox();
        hBox.setSpacing(10);
        hBox.setPadding(new Insets(10));
        hBox.getChildren().addAll(price, submit);

        VBox vBox = new VBox();
        vBox.setSpacing(10);
        vBox.setPadding(new Insets(10));
        vBox.getChildren().addAll(nameTxt, currentPrice, hBox);

        Scene scene = new Scene(vBox, 300,150);
        window.setTitle("Set Price - U1709388");
        window.setScene(scene);
        window.showAndWait();
    }
}
