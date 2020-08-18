import pyrebase

config = {
  "apiKey": "AIzaSyD6F67UAfUxa0RjO8sgQtLR9InLifIA578",
  "authDomain": "robotani-path-storage.firebaseapp.com",
  "databaseURL": "https://robotani-path-storage.firebaseio.com",
  "storageBucket": "robotani-path-storage.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

def main(path, type):
	if type==0:
		data = {"manual": path}
	else:
		data = {"auto": path}

	db.child("path").set(data)

	users = db.child("path").get()
	print(users.val()) 

#main()