import pandas as pd

folder = "4/"
timefile = folder+"t.csv"
readfiles = [folder+"xy_error.csv", folder+"a_error.csv"]
savefiles = [folder+"xy_error.csv", folder+"a_error.csv"]

csv_t_file = pd.read_csv(timefile)
for i, r_file in enumerate(readfiles):
    csv_r_file = pd.read_csv(r_file)
    csv_r_file['t'] = csv_t_file['t']
    csv_r_file.to_csv(savefiles[i], index=False)