#include <stdio.h>

// Function prototype with incorrect return type
int multiply(int a, int b) {
    return a * b;
}

// Missing return statement in non-void function
int subtract(int x, int y) {
    int result = x - y;
    // Forgot to return 'result'
}

// Using uninitialized pointer
void uninitializedPointer() {
    int *ptr;
    *ptr = 42; // Dereferencing uninitialized pointer
}

// Incorrect array index
void arrayAccess() {
    int numbers[5] = {1, 2, 3, 4, 5};
    for (int i = 0; i <= 5; i++) { // Accessing out-of-bounds index
        printf("%d ", numbers[i]);
    }
    printf("\n");
}

// Logical error in loop condition
void loopWithLogicalError() {
    int count = 5;
    while (count >= 0) { // Loop condition is incorrect
        printf("Count: %d\n", count);
        count--;
    }
}

// Division by zero
void divideByZero() {
    int numerator = 10;
    int denominator = 0;
    int result = numerator / denominator; // Division by zero
    printf("Result: %d\n", result); // This line will not be reached
}

// Incorrect use of format specifiers
void printStrings() {
    char *name = "Alice";
    int age = 30;
    printf("Name: %d, Age: %s\n", name, age); // Incorrect format specifiers
}

int main() {
    // Function call with incorrect return type
    float product = multiply(3, 4); // Expected int, assigned to float

    // Function call with missing return statement
    int difference = subtract(10, 5);

    // Function call with uninitialized pointer
    uninitializedPointer();

    // Function call with array access error
    arrayAccess();

    // Function call with logical error in loop
    loopWithLogicalError();

    // Function call causing division by zero
    divideByZero();

    // Function call with incorrect format specifiers
    printStrings();

    return 0;
}
