import os
from analyze_simulation import analyze_simulation_folder
from analyze_real import analyze_real_folder


BASE_DIR = os.path.expanduser("~/slam_analysis")
DATA_DIR = os.path.join(BASE_DIR, "data")
OUTPUT_DIR = os.path.join(BASE_DIR, "wyniki_przetworzone")


def process_scenario(scenario_name):
    scenario_input = os.path.join(DATA_DIR, scenario_name)
    scenario_output = os.path.join(OUTPUT_DIR, scenario_name)

    if not os.path.isdir(scenario_input):
        print(f"Brak folderu: {scenario_input}")
        return

    for method in ["cartographer", "slam_toolbox"]:
        method_input = os.path.join(scenario_input, method)
        method_output = os.path.join(scenario_output, method)

        if not os.path.isdir(method_input):
            print(f"Brak folderu metody: {method_input}")
            continue

        for trial_name in sorted(os.listdir(method_input)):
            input_folder = os.path.join(method_input, trial_name)
            output_folder = os.path.join(method_output, trial_name)

            if not os.path.isdir(input_folder):
                continue

            print(f"Przetwarzam: {scenario_name} / {method} / {trial_name}")

            try:
                if scenario_name == "symulacja":
                    analyze_simulation_folder(input_folder, output_folder, method, trial_name)
                elif scenario_name == "rzeczywiste":
                    analyze_real_folder(input_folder, output_folder, method, trial_name)
            except Exception as e:
                print(f"Blad w {input_folder}: {e}")


def main():
    process_scenario("symulacja")
    process_scenario("rzeczywiste")
    print("Gotowe.")


if __name__ == "__main__":
    main()
