int16 code
string message
string data
string id

int16 ERROR_INVALID_JSON     = -32700   # Invalid JSON was received by the server.
                                        # An error occurred on the server while parsing the JSON text.
int16 ERROR_INVALID_REQUEST  = -32600   # The JSON sent is not a valid Request object.
int16 ERROR_METHOD_NOT_FOUND = -32601   # The method does not exist / is not available.
int16 ERROR_INVALID_PARAMS   = -32602   # Invalid method parameter(s).
int16 ERROR_INTERNAL         = -32603   # Internal JSON-RPC error.
int16 ERROR_SERVER_FIRST     = -32000   # Reserved for implementation-defined server-errors.
int16 ERROR_SERVER_LAST      = -32099   # Reserved for implementation-defined server-errors.
