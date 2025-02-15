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