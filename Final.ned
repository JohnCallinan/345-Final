import org.omnetpp.queueing.Source;
import org.omnetpp.queueing.Classifier;
import org.omnetpp.queueing.PassiveQueue;
import org.omnetpp.queueing.Sink;
import org.omnetpp.queueing.Router;
import org.omnetpp.queueing.Server;

simple CarSource
{
    parameters:
        double interArrivalMean @unit(s);
    gates:
        output out;
}
simple TrafficLightController
{
    parameters:
        
        double durNSleft @unit(s);
        double durNSstraight @unit(s);
        double durEWleft @unit(s);
        double durEWstraight @unit(s);
    gates:
      
        input in[12];
        output ctrlOut[12];
        output out[12];
}
simple CarSink
{
    gates:
        input in;
}

simple Intersection
{
    parameters:
        double serviceTime @unit(s);  
    gates:
        input in;
        input ctrlIn;
        output out;
}
network final
{
    parameters:
        double greenDurNSleft @unit(s) = 5s;
        double greenDurNSstraight @unit(s) = 30s;
        double greenDurEWleft @unit(s) = 5s;
        double greenDurEWstraight @unit(s) = 30s;
        @display("bgb=1064,665");
    submodules:
        src[12]: CarSource {
            @display("p=140,327;is=s");
            interArrivalMean = (index==0 || index==3 || index==6 || index==9) ? 10s   : 0.5s;
        }
        tl: TrafficLightController {
            parameters:
                durNSleft = parent.greenDurNSleft;
                durNSstraight = parent.greenDurNSstraight;
                durEWleft = parent.greenDurEWleft;
                durEWstraight = parent.greenDurEWstraight;
                @display("p=380,327");
        }
        intersection[12]: Intersection {
            parameters:
                serviceTime = .2s;
                @display("p=660,327");
        }
        sink[12]: CarSink {
            @display("p=905,327;is=tr");


        }


    connections:
        for i=0..11 {
            src[i].out --> tl.in[i];
        }
        for i=0..11 {
            tl.ctrlOut[i] --> intersection[i].ctrlIn;
        }
        for i=0..11 {
            tl.out[i] --> intersection[i].in;
            intersection[i].out --> sink[i].in;
        }
}
