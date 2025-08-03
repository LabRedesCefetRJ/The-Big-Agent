teletelaUUID("b135dd8a-23e5-4b3e-9405-288c40b7fac3").
secretaryUUID("b2fc3586-245f-4c28-b1ed-56d8e7936a49").

!connect.

+!connect : teletelaUUID(UUID) <- 
	.wait(2000);
	.connectCN("skynet.chon.group", 5500, UUID);
	.print("📺: Conectado à Skynet 🌐")
.

+ready <- !pathRequest.

+!pathRequest: teletelaUUID(UUID) & secretaryUUID(Secretary) <-
	.sendOut(Secretary, tell, pathRequested(UUID));
	.print("📺: Asked route to Secretary.")
.
-!pathRequest <- .print("📺: Secretary is not reachable").

+path(Path) <-
  .send(eye, tellHow, Path);
  .wait(1000);
  .send(eye, achieve, path);
	.wait(1000);
	-path(Path)
.

+pathConcluded : teletelaUUID(UUID) & secretaryUUID(Secretary)  <- 
	.sendOut(UUID, tell, message(UUID, "Path Concluded"));
	.wait(1000);
	-pathConcluded
.
