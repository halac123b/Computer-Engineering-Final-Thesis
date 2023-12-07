def func_one():
    print("Function One")

def func_two():
    print("Function Two")

def func_three():
    print("Function Three")

def run_function_by_name(function_name):
    # Check if the function exists in the global namespace
    if function_name in globals() and callable(globals()[function_name]):
        # Get the function by name and call it
        function_to_run = globals()[function_name]
        function_to_run()
    else:
        print(f"Function '{function_name}' not found or not callable.")

# Example usage
input_function_name = input("Enter the function name: ")
run_function_by_name(input_function_name)
