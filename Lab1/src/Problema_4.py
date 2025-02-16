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