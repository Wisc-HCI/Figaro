
import eventlet
eventlet.monkey_patch()

from flask import Flask
from flask import render_template
#from flask_caching import Cache
import sys

# creates a Flask application, named app
app = Flask(__name__)
#cache = Cache()


# Can't configure the client yet...
#cache.init_app(app, {"CACHE_TYPE": "null"})

# Break convention and set options on the _client object
# directly. For pylibmc behaviors:
#cache.cache._client.behaviors["no_block"] = True
#cache.cache._client.behaviors["tcp_nodelay"] = True
#cache.cache._client.behaviors["tcp_keepalive"] = True

'''
cache.init_app(app, config={"CACHE_TYPE":"memcached",'CACHE_OPTIONS': { 'behaviors': {
                        # Faster IO
                        'tcp_nodelay': True}}})
'''
# a route where we will display a welcome message via an HTML template
@app.route("/")
def hello():
    return render_template('index.html', data = ip)

@app.route("/home")
def index():
    return render_template('index.html', data = ip)

@app.route("/sketch")
def sketch():
    return render_template('sketch.html', data = ip)

'''
@app.route("/delete")
def delete():
    return render_template('delete.html', data = ip)
'''

@app.route("/viewscene")
def deletescene():
    return render_template('viewscene.html', data = ip)

@app.route("/deploy")
def deploy():
    return render_template('deploy.html', data = ip)

@app.route("/physicallayout")
def physicallayout():
    return render_template('physicallayout.html', data = ip)

'''
@app.route("/fastforward")
def fastforward():
    return render_template('fastforward.html', data = ip)

@app.route("/insert")
def insert():
    return render_template('insert.html', data = ip)

@app.route("/modify")
def modify():
    return render_template('modify.html', data = ip)

@app.route("/pause")
def pause():
    return render_template('pause.html', data = ip)
'''

@app.route("/queryspeech")
def queryspeech():
    print("rendering template for queryspeech")
    return render_template('queryspeech.html', data = ip)

@app.route("/queryconflict")
def queryconflict():
    return render_template('queryconflict.html', data = ip)

@app.route("/editspeechcats")
def usercats():
    return render_template('editspeechcats.html', data = ip)

'''
@app.route("/rewind")
def rewind():
    return render_template('rewind.html', data = ip)
'''

@app.route("/scene")
def scene():
    return render_template('scene.html', data = ip)

# run the application
if __name__ == "__main__":
    if len(sys.argv) > 1:
        ip = sys.argv[1]
        print("host is {}".format(ip))
    else:
        ip = 'localhost'
        print("host is {}\nNOTE: to change the IP address, pass in the host as an argument".format(ip))

    #use host computer's IP address here
    #also make sure all ports used are allowed in firewall
    app.run(host=ip, port=7778)
