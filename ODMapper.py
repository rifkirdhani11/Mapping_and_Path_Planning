import os
import sys
sys.path.append('..')

from pyodm import Node, exceptions
from tkinter import *
from tkinter.ttk import Progressbar
from tkinter import ttk, filedialog
from ttkthemes import ThemedStyle 

node = Node("192.168.99.100", 3000)

value = 0

def open_file():
    global files
    files = filedialog.askopenfilenames(initialdir = "Dataset")
    print(files)
    return files

def create_task():
    try:
        #root.config(cursor='wait red red')
        # Start a task
        print("Uploading images...")
        task = node.create_task(files,
                                {'use-opensfm-dense': True,
                                 'orthophoto-resolution': 4,})
                                #,progress_callback=progressUpDisplay ,name=projNameIn)
        print(task.info(0))
        
        try:
            # This will block until the task is finished
            # or will raise an exception
            task.wait_for_completion()

            print("Task completed, downloading results...")

            # Retrieve results
            task.download_assets("./results")

            print("Assets saved in ./results (%s)" % os.listdir("./results"))

            # Restart task and this time compute dtm
            task.restart({'dtm': True})
            task.wait_for_completion()

            print("Task completed, downloading results...")

            task.download_assets("./results_with_dtm")

            print("Assets saved in ./results_with_dtm (%s)" % os.listdir("./results_with_dtm"))
        except exceptions.TaskFailedError as e:
            print("\n".join(task.output()))
        
    except exceptions.NodeConnectionError as e:
        print("Cannot connect: %s" % e)
    except exceptions.NodeResponseError as e:
        print("Error: %s" % e)

def progressUpDisplay():
    #global value
    #value=progress_callback(self)
    progress_callback(value)
    print(value)

def attributeODM(window, tab1):
    style = ThemedStyle(tab1)
    style.set_theme("vista")

    lbl1 = ttk.Label(tab1, text="Images and GCP (optional):  ")
    lbl1.grid(row=0, sticky=W)#place(x=10, y=10)
    lbl2 = ttk.Label(tab1, text="Project Name: ")
    lbl2.grid(row=1, sticky=W)#place(x=10, y=37)
    projName = ttk.Entry(tab1, width=20)
    projName.grid(row=2, sticky=W, padx=3)
    projNameIn = projName.get()

    browse = ttk.Button(tab1, text="Add Files...", command=open_file)
    browse.grid(row=0, column=1, sticky=W, pady=3)#place(x=220, y=10)
    startODM = ttk.Button(tab1, text="Start Stich", command=create_task)
    startODM.grid(row=2, column=1, sticky=W, padx=5)#place(x=220, y=37)



    