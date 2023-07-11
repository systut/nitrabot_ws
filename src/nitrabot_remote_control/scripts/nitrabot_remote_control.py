#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Standard library

# External library
import rospy

# Internal library


class NitrabotRemoteControl(object):
    """! Class for remotely controlling nitrabot using websocket client
    @param host_address<str>: host address of websocket server

	Provide different methods to manipulate robot
	"""
	# ==========================================================================
	# PUBLIC METHOD
	# ==========================================================================
    def __init__(self, host_address):

        self._host_address = host_address
        pass
    
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
        """! method to initiate connection to sio server
        return sio client
        """
        wss_address = f'wss://{self._host_address}'

        sio = socketio.Client(
            reconnection=True,
            logger=False,
            ssl_verify=False
        )

        sio.on('connect', self._on_connect, namespace=None)

        sio.on('disconnect', self._on_disconnect, namespace=None)

        sio.connect(wss_addr,
                    transports=['websocket'],
                    socketio_path="/v1/")

        sio.on('robot/{}'.format(sio.get_sid()),
               self._on_message, namespace=None)

        return sio

    def _on_message(self):
        pass

    def _on_connect(self):
        pass

    def _on_disconnect(self):
        pass

def main():
    host_address = ""

    controller = NitrabotRemoteControl(host_address)

    controller.connect()


if __name__ == "__main__":
    main()