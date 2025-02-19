# Report_1_175629

## Introduction

Python is an interpreted, high-level programming language that is renowned for being easy to learn and understand. Numerous domains, such as web development, data science, automation, artificial intelligence, and scientific computing, make extensive use of it.

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

Object-Oriented Programming (OOP) is a programming paradigm that organizes code into reusable and structured components known as objects. 
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
Encapsulation is the concept of restricting direct access to certain details of an object. It ensures that data is protected and accessed only through defined methods.

6. Inheritance
Inheritance allows a class to acquire the attributes and methods of another class. This promotes code reuse and hierarchy in object relationships.

7. Polymorphism
Polymorphism enables objects to share a common interface while having different implementations. It allows flexibility in handling different object types through a unified approach.

## Use of OOP in Python

To apply OOP in Python, developers create classes that define attributes and methods, then instantiate objects based on these classes. Encapsulation ensures controlled access to data,
 inheritance allows reusing existing class properties, and polymorphism provides flexibility by enabling different implementations of a shared behavior.

# Problems

## Problem 1

Write a program that reads a positive integer "n" entered by the user and then displays on the screen the sum of all integers from 1 to n.

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

## Conclusión

Un resumen de los puntos clave discutidos en el documento.

## Referencias

- [Referencia 1](http://ejemplo.com)
- [Referencia 2](http://ejemplo.com)