secretaryUUID("b2fc3586-245f-4c28-b1ed-56d8e7936a49").

!connect.

+!connect : secretaryUUID(UUID) <- 
	.connectCN("skynet.chon.group", 5500, UUID);
	.print("🏛️: Securency Secretary is online 😎")
.

+message(UUID, Message) <-
	.print("🏛️ ", UUID, ": ", Message)
.

+!getPath[source(Drone)] <-
	.print("🏛️: Path requested from:", Drone);
	.sendOut(Drone, achieve, path("+!path <-!takeoff; !up(20.0); !right(15.0); !forward(-45.0); !left(-1.0); !backward(0.0); !right(-1.0); !backward(1.0); !down(2.0); !land; !turnOff; !contactBack."));
.