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