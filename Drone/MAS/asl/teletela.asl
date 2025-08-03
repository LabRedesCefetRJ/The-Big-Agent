teletelaUUID("b135dd8a-23e5-4b3e-9405-288c40b7fac3").
secretaryUUID("b2fc3586-245f-4c28-b1ed-56d8e7936a49").

!connect.

+!connect : teletelaUUID(UUID) <- 
	.wait(2000);
	.connectCN("skynet.chon.group", 5500, UUID);
	.print("📺: Conectado à Skynet 🌐")
.

+ready <- !pathRequest.

+!pathRequest: teletelaUUID(UUID) & secretaryUUID(Secretary) & not running<-
	.sendOut(Secretary, achieve, getPath);
	.print("📺: Asked route to Secretary.");
	.random(R); .wait(10000*R); !pathRequest;
.

+!pathRequest: running.

-!pathRequest <- .print("📺: Secretary is not reachable").

+!path(Path) <-
  +running;
  .send(eye, tellHow, Path);
  .wait(3000);
  .send(eye, achieve, path);
.

+pathConcluded : teletelaUUID(UUID) & secretaryUUID(Secretary)  <- 
	.sendOut(UUID, tell, message(UUID, "Path Concluded"));
	.wait(1000);
	-pathConcluded
.
