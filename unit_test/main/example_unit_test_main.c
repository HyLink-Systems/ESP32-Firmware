#include <stdio.h>

// Function prototypes with incorrect types
int add(int a, b) {
    return a + b;
}

// Undefined variable usage
int main() {
    int x = 10;
    printf("The value of y is: %d\n", y); // 'y' is undefined
    return 0;
}

// Missing semicolon
void displayMessage() {
    printf("This message is missing a semicolon")
}

// Memory leaks and invalid pointer usage
void allocateMemory() {
    int *ptr = malloc(10 * sizeof(int)); // Missing #include <stdlib.h>
    *ptr = 42;
    // Forgot to free memory: memory leak
}

// Uninitialized variable usage
void uninitializedVariable() {
    int a;
    int sum = a + 5; // 'a' is uninitialized
    printf("Sum is: %d\n", sum);
}

// Buffer overflow
void bufferOverflow() {
    char buffer[5];
    strcpy(buffer, "Hello, world!"); // Buffer overflow: string is too long
}

// Infinite loop
void infiniteLoop() {
    int i;
    while (1) {
        i++; // 'i' is not initialized, leads to undefined behavior
    }
}

// Incorrect function return type
double calculateAverage(int a, int b) {
    return a + b; // Should return (a + b) / 2.0
}

// Using incorrect format specifier
void printChar() {
    char ch = 'A';
    printf("Character: %d\n", ch); // Incorrect format specifier for char
}

int main() {
    // Redefinition of 'main' function
    printf("This is a redefined main function\n");
    return 0;
}
