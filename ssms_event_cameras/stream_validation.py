import os
import time
import shutil
import subprocess
import multiprocessing

# Configura i percorsi e il comando
target_directory = '/home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz/test'
watch_directory = '/home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz/val'
command = 'python /home/salvatore/ssms_event_cameras/RVT/validation.py dataset=gen1 dataset.path="/home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz" checkpoint="/home/salvatore/ssms_event_cameras/checkpoint/gen1_small.ckpt" use_test_set=1 hardware.gpus=0 +experiment/gen1="small.yaml" batch_size.eval=1 model.postprocess.confidence_threshold=0.01 hardware.num_workers.eval=1'

def has_valid_structure(dir_path):
    """Verifica che la directory abbia la struttura richiesta."""
    event_representation_base_path = os.path.join(dir_path, 'event_representations_v2')
    new_folder_name = 'stacked_histogram_ne=25000_nbins=10'
    target_folder_path = os.path.join(event_representation_base_path, new_folder_name)
    labels_path = os.path.join(dir_path, 'labels_v2')

    # Rinomina una specifica sotto-cartella all'interno di event_representations_v2
    if os.path.isdir(event_representation_base_path):
        # Trova una sotto-cartella specifica da rinominare (esempio: la prima sotto-cartella trovata)
        sub_folders = [d for d in os.listdir(event_representation_base_path) if os.path.isdir(os.path.join(event_representation_base_path, d))]
        if sub_folders:  # Assicurati che ci sia almeno una sotto-cartella
            specific_sub_folder = os.path.join(event_representation_base_path, sub_folders[0])  # Prende la prima sotto-cartella
            os.rename(specific_sub_folder, target_folder_path)
        else:
            print("Nessuna sotto-cartella trovata per la rinominazione.")
            return False

    required_files = [
        os.path.join(target_folder_path, 'event_representations.h5'),
        os.path.join(target_folder_path, 'objframe_idx_2_repr_idx.npy'),
        os.path.join(target_folder_path, 'timestamps_us.npy'),
        os.path.join(labels_path, 'labels.npz'),
        os.path.join(labels_path, 'timestamps_us.npy')
    ]
    print(all(os.path.isfile(file) for file in required_files))
    return all(os.path.isfile(file) for file in required_files)


def worker():
    # Lista delle directory presenti
    dirs = [d for d in os.listdir(watch_directory) if os.path.isdir(os.path.join(watch_directory, d))]
    # Filtra le directory che contengono la struttura richiesta
    valid_dirs = []
    for d in dirs:
        dir_path = os.path.join(watch_directory, d)
        if has_valid_structure(dir_path):
            valid_dirs.append(d)
    print(valid_dirs)
    
    if valid_dirs:
        print(f"Trovate directory valide: {valid_dirs}")
        moved_dirs = []
        
        # Sposta tutte le directory valide nel percorso di destinazione
        for d in valid_dirs:
            source_dir_path = os.path.join(watch_directory, d)
            target_dir_path = os.path.join(target_directory, d)
            try:
                shutil.move(source_dir_path, target_dir_path)
                print(f"Spostata la directory {source_dir_path} a {target_dir_path}")
                moved_dirs.append(target_dir_path)
            except Exception as e:
                print(f"Errore durante lo spostamento della directory {source_dir_path}: {e}")
        
        # Esegui il comando e poi elimina ogni directory spostata
        subprocess.run(command, shell=True, check=True)
        print(f"Comando eseguito con successo nella directory: {dir_path}")

        for dir_path in moved_dirs:
            try:
                shutil.rmtree(dir_path)
                print(f"Directory eliminata con successo: {dir_path}")
            except Exception as e:
                print(f"Errore nell'eliminare la directory {dir_path}: {e}")
    
   
if __name__ == "__main__":
    while True:
        # Controlla se la directory target è vuota
        if not os.listdir(target_directory):
            print("Directory target vuota, avvio del processo di monitoraggio.")
            process = multiprocessing.Process(target=worker)
            process.start()
            
        else:
            print("Directory target non vuota, attendo...")
        
        time.sleep(5)  # controlla ogni 10 secondi se la directory target è vuota
