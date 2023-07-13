#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Standard library
import json
import threading

# External library
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

    def connect(self, callback):
        """! Connect to websocket server
        @param callback<function>: callback function when receiving message
        """
        self._client = self._define_client(callback)

        client_thread = threading.Thread(target=self._client.wait)

        client_thread.setDaemon(True)

        client_thread.start()

	# ==========================================================================
	# PRIVATE METHOD
	# ==========================================================================
    def _define_client(self, callback):
        """! Initiate connection to sio server
        @param callback<function>: callback function when receiving message
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

        client.connect(wss_address,
                    transports=['websocket'],
                    socketio_path="/v1/")

        client.on('robot/{}'.format(client.get_sid()), 
        callback, namespace=None)

        return client
    
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
