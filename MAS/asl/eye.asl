serialPort(ttyEmulatedPort0).

!start.

// 👁️ ⬆️ ⬇️ ➡️ ⬅️ 

//--------------- Loops Core ---------------

+!start <-
    .print("--: Opening Eyes...");
	.argo.port(ttyEmulatedPort0);
	.argo.percepts(open);
	.argo.limit(1000);
    .print("👁️: Body connected.");
	.send(teletela, tell, ready);
    .wait(2000);
.

-!path <- 
  .print("👁️: I do not know the path");
.

//--------------- Fase de navegação ---------------

+!takeoff <- 
	.print("👁️: To infinity and beyond!");
	
	.argo.act(up);

	.wait(3000);

	.argo.act(up);

	.wait(2000);

	.argo.act(up);

	.wait(2000);

	+flying;

	.print("👁️: Ready to action!")
.

+!land : flying <- 
    .print("👁️: Landing...");
    .argo.act(land);
    -flying
.

+!turnOff <- 
    .print("👁️: Turning off!");
    .argo.act(off);
    .wait(1000);
    .print("--: Closing eyes!");
.

//--------------- UP ---------------

+!up(Limit): flying <-
    .print("👁️: Rising to: ", Limit);
    !rising(Limit)
.

+!rising(Limit): gps(_, _, Z) & (1 * Limit) - 0.1 > Z  <- 
    //!printMetaDebug(Limit); //! debug
	
    .print("👁️: Rising... ⬆️ ");
        .argo.act(up);

    .wait(2000);
    !rising(Limit)
.
-!rising(Limit): gps(_, _, Z) & (1 * Limit) - 0.1 > Z  <- 
	.wait(2000);
	!rising(Limit)
.


+!rising(Limit) : gps(_, _, Z) & (1 * Limit) - 0.1 <= Z <- 
   	.print("👁️: Rise concluded.");
.



//--------------- DOWN ---------------

+!down(DownLimit) : flying <-
    .print("👁️: Downing to: ", DownLimit);
    !downing(DownLimit)
.

+!downing(DownLimit) : gps(_, _, Z) & (DownLimit + 0.1) < Z <- 
    //!printMetaDebug(DownLimit); //! debug
    
    .print("👁️: Downing... ⬇️");
    .argo.act(down);

    .wait(2000);
    !downing(DownLimit)
.

-!downing(DownLimit) : gps(_, _, Z) & (DownLimit + 0.1) < Z <- 
    .wait(2000);
    !downing(DownLimit)
.

+!downing(DownLimit) : gps(_, _, Z) & (DownLimit + 0.1) >= Z <- 
    .print("👁️: Descida concluída.");
.



//--------------- Foward X ---------------

+!forward(FrontLimit) : flying <-
    .print("👁️: Indo para frente até: ", FrontLimit);
    !forwarding(FrontLimit)
.


+!forwarding(FrontLimit) : gps(X, _, _) & (FrontLimit + 0.1) < X <- 
    //!printMetaDebug(FrontLimit); //! debug

    .print("👁️: Going To Destination!");
    .argo.act(forward);

    .wait(2000);
    !forwarding(FrontLimit)
.
-!forwarding(FrontLimit) : gps(X, _, _) & (FrontLimit + 0.1) < X <- 
    .wait(2000);
    !forwarding(FrontLimit)
.


+!forwarding(FrontLimit) : gps(X, _, _) & (FrontLimit + 0.1) >= X <- 
    .print("👁️: Frente concluída.");
.


//--------------- Backwards X ---------------

+!backward(BackLimit) : flying <-
    .print("👁️: Indo para trás até: ", BackLimit);
    !backwarding(BackLimit)
.

+!backwarding(BackLimit) : gps(X, _, _) & (BackLimit - 0.1) > X <- 
    //!printMetaDebug(BackLimit); //! debug

    .print("👁️: Indo para trás ⬇️");
    .argo.act(backward);

    .wait(2000);
    !backwarding(BackLimit)
.

-!backwarding(BackLimit) : gps(X, _, _) & (BackLimit - 0.1) > X <- 
    .wait(2000);
    !backwarding(BackLimit)
.

+!backwarding(BackLimit) : gps(X, _, _) & (BackLimit - 0.1) <= X <- 
    .print("👁️: Traseira concluída.");
.

//--------------- Left Y ---------------

+!left(LeftLimit) : flying <-
    .print("👁️: Indo para esquerda até: ", LeftLimit);
    !lefting(LeftLimit)
.

+!lefting(LeftLimit) : gps(_, Y, _) & (LeftLimit + 0.1) < Y <- 
    //!printMetaDebug(LeftLimit); //! debug

    .print("👁️: Indo para esquerda ⬅️");
    .argo.act(left);

    .wait(2000);
    !lefting(LeftLimit)
.

-!lefting(LeftLimit) : gps(_, Y, _) & (LeftLimit + 0.1) < Y <- 
    .wait(2000);
    !lefting(LeftLimit)
.

+!lefting(LeftLimit) : gps(_, Y, _) & (LeftLimit + 0.1) >= Y <- 
    .print("👁️: Esquerda concluída.");
.


//--------------- Right Y ---------------

+!right(RightLimit) : flying <-
    .print("👁️: Indo para direita até: ", RightLimit);
    !righting(RightLimit)
.

+!righting(RightLimit) : gps(_, Y, _) & (RightLimit - 0.1) > Y <- 
    //!printMetaDebug(RightLimit); //! debug

    .print("👁️: Indo para direita ➡️");
    .argo.act(right);

    .wait(2000);
    !righting(RightLimit)
.

-!righting(RightLimit) : gps(_, Y, _) & (RightLimit - 0.1) > Y <- 
    .wait(2000);
    !righting(RightLimit)
.

+!righting(RightLimit) : gps(_, Y, _) & (RightLimit - 0.1) <= Y <- 
    .print("👁️: Direita concluída.")
.

//--------------- Comunicate ---------------

+!contactBack <- 
	.send(teletela, tell, pathConcluded).

//--------------- Debug ---------------


//+gps(X, Y, Z) <- 
	//.print("--DEBUG GPS: Percepção recebida: gps(", X, ", ", Y, ", ", Z, ")")
//.

+!printMetaDebug(A) : gps(X, Y, Z) <-
   .print("--DEBUG: Position X: ", X, " Meta: ", ((A * 1) - 0.1) );
   .print("--DEBUG: Position Y: ", Y, " Meta: ", ((A * 1) - 0.1) );
   .print("--DEBUG: Position Z: ", Z, " Meta: ", ((A * 1) - 0.1) );
   .wait(1500)
.
