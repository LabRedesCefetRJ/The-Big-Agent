secretaryUUID("b2fc3586-245f-4c28-b1ed-56d8e7936a49").

!connect.

+!connect : secretaryUUID(UUID) <- 
	.connectCN("skynet.chon.group", 5500, UUID);
	.print("🏛️: Security Secretary is online 😎 ")
.

+message(UUID, Message) <-
	.print("🏛️", UUID, ": ", Message)
.

+pathRequested(AgentUUID) <-
	.print("🏛️: Path requested from:", AgentUUID);
	!generateRandomNumber(Result);
	!getRandomPath(Result, AgentUUID);
  .wait(1000);
  -pathRequest(AgentUUID)
.


// -------------------- Random Path  --------------------

+!generateRandomNumber(Result) <-
	.random(RandomNumber);
	Result = RandomNumber * 10;
.

+!getRandomPath(Result, AgentUUID) : Result > 6 | Result < 1 <-
	!generateRandomNumber(R);
	!getRandomPath(R, AgentUUID)
.

+!getRandomPath(Result, AgentUUID) : Result > 1 & Result < 2  <- 
	.sendOut(AgentUUID, tell, path("+!path <-!takeoff; !up(20.0); !right(15.0); !forward(-45.0); !left(-1.0); !backward(0.0); !right(-1.0); !backward(1.0); !down(2.0); !land; !turnOff; !contactBack."));
	.print("🏛️: Path 1 selected");
.

+!getRandomPath(Result, AgentUUID) : Result > 2 & Result < 3  <-
 	.sendOut(AgentUUID, tell, path("+!path <-!takeoff; !up(20.0); !right(15.0); !forward(-45.0); !left(-1.0); !backward(0.0); !right(-1.0); !backward(1.0); !down(2.0); !land; !turnOff; !contactBack."));
	.print("🏛️:: Path 2 selected");
.

+!getRandomPath(Result, AgentUUID) : Result > 3 & Result < 4  <- 
	.sendOut(AgentUUID, tell, path("+!path <-!takeoff; !up(20.0); !right(15.0); !forward(-45.0); !left(-1.0); !backward(0.0); !right(-1.0); !backward(1.0); !down(2.0); !land; !turnOff; !contactBack."));
	.print("🏛️:: Path 3 selected");
.

+!getRandomPath(Result, AgentUUID) : Result > 4 & Result < 5  <- 
	.sendOut(AgentUUID, tell, path("+!path <-!takeoff; !up(20.0); !right(15.0); !forward(-45.0); !left(-1.0); !backward(0.0); !right(-1.0); !backward(1.0); !down(2.0); !land; !turnOff; !contactBack."));
	.print("🏛️:: Path 4 selected");
.


+!getRandomPath(Result, AgentUUID) : Result > 5 & Result < 6  <- 
	.sendOut(AgentUUID, tell, path("+!path <-!takeoff; !up(20.0); !right(15.0); !forward(-45.0); !left(-1.0); !backward(0.0); !right(-1.0); !backward(1.0); !down(2.0); !land; !turnOff; !contactBack."));
	.print("🏛️:: Path 5 selected");
.
