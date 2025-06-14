// Agent bob in project .argoAgent
serialPort(ttyEmulatedPort0).
/* Initial goals */
!start.
/* Plans */
+!start: serialPort(Port) <- 
	.argo.port(Port); 
	.argo.percepts(open);  
	.argo.limit(750);   
	!run.

+!run: dLeft(DL) & dRight(DR) & (DR >= 20) & (DL >= 20) & not wall <- 
	.argo.act(goAhead); 
	.wait(500); 
	!run.

+!run: dLeft(DL) & dRight(DR) & ((DR < 20) | (DL <20)) & ((DR >= 10) | (DL >=10)) & not wall <- 
	.argo.act(stop); 
	+wall; 
	.wait(500); 
	!run.

+!run: dLeft(DL) & dRight(DR) & ((DR < 10) | (DL <10)) & not wall <- 
	.argo.act(goBack); 
	+wall; 
	.wait(500); 
	!run.

+!run: dLeft(DL) & dRight(DR) & wall & ((DR < 30) | (DL <30) | (DR>100) | (DL>100)) <- 
	.argo.act(goRight); 
	.wait(500); 
	!run.

+!run: dLeft(DL) & dRight(DR) & wall & ((DR >= 30) | (DL >= 30)) <- 
	-wall; 
	!run.

-!run.

+port(Port,Status): (Status=timeout) | (Status=off)<-
	.argo.percepts(close); .abolish(port(_,_));
	.wait(5000).

+dLeft(DL)  <- .print("Left Distance: ",DL).
+dRight(DR) <- .print("Right Distance: ",DR).
