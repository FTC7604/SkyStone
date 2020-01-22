import javafx.scene.control.Alert;
import java.io.Serializable;

public class Product implements Serializable{
    
    // The attributes
    private String name;
    private int stockLevel;
    private double price;

    // The constructor
    public Product(String nameIn, int stockLevelIn, double priceIn)
    {
        name = nameIn;
        stockLevel = stockLevelIn;
        price = priceIn;
    }

    public void reStock(int stockIn)
    {
        stockLevel = stockLevel + stockIn;
    }
    public double sell(int quantityIn)
    {
        stockLevel = stockLevel - quantityIn;
        return quantityIn;
    }
    public void setPrice(double priceIn)
    {
        price = priceIn;
    }
    public String getName()
    {
        return name;
    }
    public int getStockLevel()
    {
        return stockLevel;
    }
    public double getPrice()
    {
        return price;
    }
}
