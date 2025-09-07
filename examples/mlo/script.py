import subprocess
import sys

def run_agent_n_times(n):
    for i in range(n):
        print(f"\n--- Run {i + 1} ---")
        try:
            # Run the agent script as a subprocess
            result = subprocess.run(
                ["python3", "agent_0-1_action.py", f"--seed={41+i}"],
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"Run {i + 1} failed with exit code {e.returncode}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 script.py <number_of_runs>")
        sys.exit(1)

    try:
        n = int(sys.argv[1])
        run_agent_n_times(n)
    except ValueError:
        print("Please provide a valid integer for the number of runs.")
