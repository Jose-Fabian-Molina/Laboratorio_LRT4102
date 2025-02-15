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