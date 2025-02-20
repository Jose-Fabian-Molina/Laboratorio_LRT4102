    # Class Producto stores details of a product
class Producto:
    # Initialize product attributes: nombre, precio, and cantidad_en_stock
    def __init__(self, nombre, precio, cantidad_en_stock):
        self.nombre = nombre
        self.precio = precio
        self.cantidad_en_stock = cantidad_en_stock

    # Method to reduce stock when selling a product
    def vender(self, cantidad_vendida):
        if cantidad_vendida <= self.cantidad_en_stock:
            self.cantidad_en_stock -= cantidad_vendida
            print(f"Selling {cantidad_vendida} units of {self.nombre}.")
        else:
            print(f"Not enough stock for {self.nombre}.")

    # Method to return product information as a formatted string
    def mostrar_informacion(self):
        return f"Product: {self.nombre} | Price: {self.precio} | Stock: {self.cantidad_en_stock}"


# Class Inventario manages a collection of Producto objects
class Inventario:
    # Initialize the inventory with an empty list of products
    def __init__(self):
        self.productos = []

    # Method to add a new product to the inventory
    def agregar_producto(self, producto):
        self.productos.append(producto)

    # Method to update stock by selling a specified quantity of a product
    def actualizar_stock(self, nombre, cantidad_vendida):
        producto = self.encontrar_producto(nombre)
        if producto:
            producto.vender(cantidad_vendida)
        else:
            print(f"Product '{nombre}' not found in inventory.")

    # Method to display information for a specific product
    def mostrar_informacion_producto(self, nombre):
        producto = self.encontrar_producto(nombre)
        if producto:
            print(producto.mostrar_informacion())
        else:
            print(f"Product '{nombre}' not found in inventory.")

    # Method to calculate the total value of the inventory (price multiplied by stock)
    def calcular_valor_inventario(self):
        valor_total = 0
        for producto in self.productos:
            valor_total += producto.precio * producto.cantidad_en_stock
        return valor_total

    # Helper method to find a product by its name (case-insensitive)
    def encontrar_producto(self, nombre):
        for producto in self.productos:
            if producto.nombre.lower() == nombre.lower():
                return producto
        return None


# Main function provides a menu-driven interface for inventory management
def main():
    inventario = Inventario()

    while True:
        # Display menu options
        print("\n--- Inventory Management ---")
        print("1. Add product")
        print("2. Sell product")
        print("3. Show product info")
        print("4. Show total inventory value")
        print("5. Exit")

        opcion = input("Enter your choice: ")

        # Option 1: Add a new product to the inventory
        if opcion == "1":
            nombre = input("Enter product name: ")
            precio = float(input("Enter product price: "))
            cantidad = int(input("Enter product stock quantity: "))
            nuevo_producto = Producto(nombre, precio, cantidad)
            inventario.agregar_producto(nuevo_producto)
            print(f"Product '{nombre}' added to inventory.")

        # Option 2: Sell a product, updating its stock
        elif opcion == "2":
            nombre = input("Enter product name to sell: ")
            cantidad_vendida = int(input("Enter quantity to sell: "))
            inventario.actualizar_stock(nombre, cantidad_vendida)

        # Option 3: Display information about a specific product
        elif opcion == "3":
            nombre = input("Enter product name to view info: ")
            inventario.mostrar_informacion_producto(nombre)

        # Option 4: Calculate and show the total value of the inventory
        elif opcion == "4":
            valor_total = inventario.calcular_valor_inventario()
            print(f"Total inventory value: {valor_total}")

        # Option 5: Exit the program
        elif opcion == "5":
            print("Exiting the program.")
            break

        # Handle invalid menu choices
        else:
            print("Invalid choice. Please try again.")


# Entry point of the program
if __name__ == "__main__":
    main()