import os
from flask import Flask, jsonify, abort, request

import ObstacleRL 

app = Flask(__name__)

@app.route('/api', methods=['POST'])
def env():

    data = request.get_json(force = True)
    world = ObstacleRL.world()
    reward,state = world.move(data['action'])
    
    return jsonify({'reward':reward,'observation':state})
    

if __name__ == '__main__':
    app.run(host='192.168.1.136',port=9000, debug=True)
