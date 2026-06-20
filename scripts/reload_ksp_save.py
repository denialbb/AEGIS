"""Connect to kRPC and load a KSP save file, then exit."""
import krpc

conn = krpc.connect(name="SaveReloader", address="172.22.80.1", rpc_port=50000, stream_port=50001)
sc = conn.space_center

sc.load("aegis_tune_start")
print("Loaded quicksave")
conn.close()
