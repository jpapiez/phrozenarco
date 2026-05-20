from .cmds import *


class Apis(Commands):
    # constructor initialization
    def __init__(self, config):
        super(Apis, self).__init__(config)
        self.G_WebsocketWebhooks = self.G_PhrozenPrinter.lookup_object("webhooks")

    # register WebSocket API
    def WebsocketAPIs_RegisterAPIs(self):
        self.G_PhrozenFluiddRespondInfo(
            "[(dev.python)WebsocketAPIs_RegisterAPIs]Registering Phrozen custom WebSocket API endpoints"
        )
        self.G_WebsocketWebhooks.register_endpoint(
            "phrozen/soft_ver", self.WebsocketAPIs_SoftVersion
        )

    def WebsocketAPIs_SoftVersion(self, web_request):
        self.G_PhrozenFluiddRespondInfo("[(dev.python)WebsocketAPIs_SoftVersion]")
        # WebSocket send
        web_request.send({"soft_version": FW_VERSION})
