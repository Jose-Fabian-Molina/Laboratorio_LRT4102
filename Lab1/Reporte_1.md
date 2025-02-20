# Report_1_175629

## Introduction

Python is an interpreted, high-level programming language that is renowned for being easy to learn and understand. Numerous domains, 
such as web development, data science, automation, artificial intelligence, and scientific computing, make extensive use of it.

### Variable types

In Python, variables store different types of data without requiring explicit type declarations. Some common types include:

- Integers: Whole numbers without decimals.
- Floats: Numbers that contain decimal points.
- Strings: A sequence of characters, often used to represent text.
- Booleans: Logical values that can be either True or False.
- Lists: Ordered collections that can store multiple values of different types.
- Dictionaries: Collections of key-value pairs, used for structured data storage.

### Control structures

Python provides several control structures to manage the flow of execution.

#### For loop

A for loop is used for iterating over sequences such as lists, ranges, or strings. It allows executing a block of code multiple times.

#### While loop

A while loop executes a block of code as long as a specified condition remains true. It is useful when the number of iterations is not 
known in advance.

Python's intuitive syntax and dynamic typing make it an accessible and powerful programming language. Its control structures, such as 
loops, help manage repetitive tasks efficiently, making it a preferred choice for a wide range of applications.

# Object-Oriented Programming (OOP) in Python

Object-Oriented Programming (OOP) is a programming paradigm that organizes code into reusable and structured components known as 
objects. 
Python supports OOP, allowing developers to create and manage complex systems efficiently.

## Key Components of OOP

1. Clasess
A class is a blueprint for creating objects. It defines attributes (characteristics) and methods (behaviors) that the objects will have.

2. Objects
An object is an instance of a class. Each object has unique attributes and can perform specific actions defined in the class.

3. Attributes
Attributes represent the properties or data associated with an object. They define the state of an object.

4. Methods
Methods are functions inside a class that define the behavior of objects. They allow objects to interact with data and perform actions.

5. Encapsulation
Encapsulation is the concept of restricting direct access to certain details of an object. It ensures that data is protected and 
accessed only through defined methods.

6. Inheritance
Inheritance allows a class to acquire the attributes and methods of another class. This promotes code reuse and hierarchy in object 
relationships.

7. Polymorphism
Polymorphism enables objects to share a common interface while having different implementations. It allows flexibility in handling 
different object types through a unified approach.

## Use of OOP in Python

To apply OOP in Python, developers create classes that define attributes and methods, then instantiate objects based on these classes. 
Encapsulation ensures controlled access to data, inheritance allows reusing existing class properties, and polymorphism provides 
flexibility by enabling different implementations of a shared behavior.

# Problems

## Problem 1

Write a program that reads a positive integer "n" entered by the user and then displays on the screen the sum of all integers from 1 to 
n.

```python
def main():

    n = int(input("Introduce un entero positivo: ")) # The user introduces the number to apply the addition to.

    suma = n * (n + 1) // 2 

    print(f"La suma de los primeros {n} enteros positivos es: {suma}") # Print the addition.

if __name__ == "__main__":
    main()
```

The solution involves reading a positive integer n from the user and calculating the sum of all integers from 1 to n. Instead of using 
a loop to add each number sequentially, we utilize a mathematical formula for summing the first n positive integers:

Sum = (n(n+1))/2

## Problem 2

Write a program that asks the user for the number of hours worked and the hourly rate. Then, it should display the corresponding pay on 
the screen.

```python
def main():
    # Ask the user for the number of hours worked
    horas_trabajadas = float(input("Enter the number of hours worked: "))

    # Ask the user for the hourly rate
    costo_por_hora = float(input("Enter the hourly rate: "))

    # Calculate the total pay
    paga_total = horas_trabajadas * costo_por_hora

    # Display the result
    print(f"Your total pay is: $ {paga_total}")

if __name__ == "__main__":
    main()
```

The solution involves asking the user to input two values:

- Number of hours worked
- Hourly rate (cost per hour)

Once both values are provided, the total pay is calculated using the formula:

Total Pay = Hours Worked × Hourly Rate

## Problem 3

Create a list with the name, hourly wage, and hours worked for at least six operators.  
Print the name and the total pay for each operator.

```python
def main():
    # Create a list of operators (name, hourly wage, hours worked)
    operadores = [
        ("Alice", 15.0, 40),
        ("Bob", 18.0, 35),
        ("Charlie", 20.0, 30),
        ("Diana", 22.5, 38),
        ("Edward", 17.5, 42),
        ("Fiona", 19.0, 36)
    ]
    
    # Iterate over the list and calculate the total pay for each operator
    for operador in operadores:
        nombre, sueldo_por_hora, horas_trabajadas = operador
        sueldo_a_pagar = sueldo_por_hora * horas_trabajadas
        print(f"Operator: {nombre}, Total pay: $ {sueldo_a_pagar}")

if __name__ == "__main__":
    main()
```

The program stores data for at least six operators in a list, where each operator has a name, hourly wage, and hours worked. It then 
iterates through the list, calculating each operator's total pay by multiplying their hourly wage by their hours worked. Finally, it 
prints each operator's name along with their corresponding total pay.

## Problem 4

- Create a list called **numeros** that contains at least 10 numbers.  
- Calculate the average of the even numbers and the product of the odd numbers.  
- Print the results.

```python
def main():
    # Create a list of at least 10 numbers
    numeros = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    
    # Filter even numbers and calculate their average
    pares = [num for num in numeros if num % 2 == 0]
    promedio_pares = sum(pares) / len(pares) if pares else 0
    
    # Filter odd numbers and calculate their product
    impares = [num for num in numeros if num % 2 != 0]
    producto_impares = 1
    for num in impares:
        producto_impares *= num
    
    # Print the results
    print(f"Average of even numbers: {promedio_pares}")
    print(f"Product of odd numbers: {producto_impares}")

if __name__ == "__main__":
    main()
```

The program starts by creating a list called numeros with at least 10 numbers. It then separates the even and odd numbers from the 
list. To find the average of the even numbers, it sums them up and divides by the count of even numbers. For the product of the odd 
numbers, it multiplies all the odd numbers together. Finally, it prints both the calculated average and the product.

## Problem 5

Create a program that asks the user to guess a secret number. The program should generate a random number between 1 and 10, and the user must try to guess it. The program should provide hints if the user's guess is too high or too low. The **while** loop should continue until the user guesses correctly. In the end, the program should print how many attempts it took for the user to guess the number.

```python
import random

def main():
    # Generate a secret number between 1 and 10
    numero_secreto = random.randint(1, 10)
    intentos = 0
    adivinado = False

    # Loop until the user guesses the correct number
    while not adivinado:
        # Ask the user to guess the secret number
        guess = int(input("Guess the secret number between 1 and 10: "))
        intentos += 1

        if guess < numero_secreto:
            print("Too low!")
        elif guess > numero_secreto:
            print("Too high!")
        else:
            print(f"Correct! You guessed the number in {intentos} attempts.")
            adivinado = True

if __name__ == "__main__":
    main()
```

The program generates a random secret number between 1 and 10. It then prompts the user to guess the number, providing hints if the 
guess is too high or too low. A while loop ensures that the guessing process continues until the correct number is found. The program 
keeps track of the number of attempts and, once the user guesses correctly, it displays the total number of tries taken.

## Problem 6

The program should generate a matrix of at least **5x5**.  
The robot starts its path at position **(0,0)** in the matrix and must exit at position **(4,4)** or the maximum position if the matrix 
size is changed.  

- The number and position of obstacles are random.  
- The robot can only move forward, turn left, or turn right to find a free path.  
- If the robot cannot reach the destination, it should print **"Impossible to reach the destination"** on the screen.  
- If the robot successfully reaches its final destination, it should print the map with free spaces and obstacles displayed as 
follows:  

```
o o o X o  
o o o o o  
o o o o X  
o o o o o  
o X X X o  
```

Additionally, the program should print the **route** the robot followed.  
A second map should be displayed showing the **path** taken by the robot using arrows.

### Solution

The program starts by importing `random` and `numpy` to handle matrix creation and obstacle placement. It prompts the user for a matrix 
size (ensuring it is at least **5x5**) and initializes two matrices: **one for the environment (`m`)** and **one for tracking movement 
(`d`)**. The matrix is filled with **random obstacles (`"X"`)** and open spaces (`"o"`), with the **robot starting at (0,0) (`"R"`)** 
and the **destination at the bottom-right (`"D"`)**.

The robot moves using a `while` loop, following a **priority-based movement strategy**: it first tries to **move right**, then 
**down**, updating its position and marking the path in the direction matrix (`d`) using arrows (`→`, `↓`). If both right and down are 
blocked, the program **backtracks by resetting the robot’s position to the next row** and trying again. If no valid path is found after 
exploring all options, the program prints **"No valid path exists"** and terminates. If the robot **successfully reaches the goal**, it 
prints both matrices: **one showing obstacles and free spaces, and another displaying the path taken with arrows**. Finally, the 
**total number of moves** taken is displayed.
 
The program simulates a **pathfinding algorithm in a grid with obstacles**, ensuring the robot finds a valid path or determines if 
reaching the goal is impossible. It uses **nested loops for initialization**, a **while loop for movement**, and **conditional 
statements (`if-elif`)** to control direction. The use of **randomized obstacles** adds unpredictability, and the program efficiently 
**tracks the robot's journey** by updating the matrix with directional arrows.

# Problem 7

This solution employs the Object-Oriented Programming (OOP) paradigm to implement a simple inventory management system. The design is 
structured around two primary classes: Producto and Inventario, which encapsulate the behavior and data for individual products and the 
overall inventory, respectively. A menu-driven interface in the main() function allows the user to interact with the system.

## Key components

### 1. `Producto` Class
- **Purpose:**  
  Represents a single product within the inventory.
  
- **Attributes:**  
  - `nombre`: The name of the product.  
  - `precio`: The price per unit of the product.  
  - `cantidad_en_stock`: The current stock level or quantity available.

- **Methods:**  
  - `vender(cantidad_vendida)`:  
    Reduces the product’s stock by the specified amount if sufficient stock is available. If not, it prints a message indicating insufficient stock.
  - `mostrar_informacion()`:  
    Returns a formatted string that contains the product’s details (name, price, and stock).

### 2. `Inventario` Class
- **Purpose:**  
  Manages a collection of `Producto` objects and provides functionality to update and retrieve inventory data.
  
- **Attributes:**  
  - `productos`: A list that stores instances of the `Producto` class.
  
- **Methods:**  
  - `agregar_producto(producto)`:  
    Adds a new `Producto` object to the inventory list.
  - `actualizar_stock(nombre, cantidad_vendida)`:  
    Searches for a product by its name and reduces its stock using the `vender` method if found.
  - `mostrar_informacion_producto(nombre)`:  
    Displays detailed information for a specific product by calling its `mostrar_informacion` method.
  - `calcular_valor_inventario()`:  
    Calculates the total value of the inventory by summing the product of each product’s price and its available stock.
  - `encontrar_producto(nombre)`:  
    A helper method that performs a case-insensitive search for a product in the inventory by its name.

### 3. `main()` Function
- **Purpose:**  
  Serves as the entry point of the program, offering a user interface to interact with the inventory system.
  
- **Flow:**  
  - An instance of the `Inventario` class is created.
  - The program enters a loop where it displays a menu with options to:
    - Add a product
    - Sell a product
    - Show product information
    - Calculate and display the total inventory value
    - Exit the program
  - Based on the user’s choice, the corresponding methods of the `Producto` or `Inventario` classes are invoked to perform the required operations.
  - The loop continues until the user chooses to exit.