import matplotlib.pyplot as plt
import csv

class DataHandler:
    def save_to_csv(self, data, fields, path):
        with open(path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(fields)
            writer.writerows(data)
        
        