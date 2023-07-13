#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Standard library
import threading

# External library
import rospy
import socketio

# Internal library


class WebsocketClient(object):
    """! Class for remotely controlling nitrabot using websocket client
    @param host_address<str>: host address of websocket server

	Provide different methods to manipulate robot
	"""
	# ==========================================================================
	# PUBLIC METHOD
	# ==========================================================================
    def __init__(self, host_address):

        self._host_address = host_address

        self._mutex_lock = threading.Lock()

    def connect(self):
        """! Connect to websocket server
        """
        self._client = self._define_client()

        client_thread = threading.Thread(target=self._client.wait)

        client_thread.setDaemon(True)

        client_thread.start()

	# ==========================================================================
	# PRIVATE METHOD
	# ==========================================================================
    def _define_client(self):
        """! Initiate connection to sio server
        @return sio: socketio instance 
        """
        wss_address = f'wss://{self._host_address}'

        client = socketio.Client(
            reconnection=True,
            logger=False,
            ssl_verify=False
        )

        client.on('connect', self._on_connect, namespace=None)

        client.on('disconnect', self._on_disconnect, namespace=None)

        client.connect(wss_addr,
                    transports=['websocket'],
                    socketio_path="/v1/")

        client.on('robot/{}'.format(client.get_sid()), 
        self._on_message, namespace=None)

        return sio
    
    def _on_message(self, msg):
        """! Callback for receiving message event
        @param msg<str> received message 
        """
        json_msg = json.loads(msg)

    def _on_connect(self):
        """! Callback for connect event
        """
        self.is_connect_ = True

    def _on_disconnect(self):
        """! Callback for disconnect event
        """
        self.is_connect_ = False

    def _send_message(self, msg):
        """! Send message to websocket server
        @param msg<dict> message that sent to server 
        """
        message = json.dumps(msg)

        self._mutex_lock.acquire()

        self._client.emit('robot', message, namespace=None)

        self._mutex_lock.release()

def main():
    host_address = ""

    controller = NitrabotRemoteControl(host_address)

    controller.connect()


if __name__ == "__main__":
    main()