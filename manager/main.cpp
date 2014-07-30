#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <deque>
#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Port.h>
#include <yarp/os/Vocab.h>

#define VOCAB_CMD_HOME           VOCAB4('h','o','m','e')
#define VOCAB_CMD_BLOCK          VOCAB4('b','l','o','c')
#define VOCAB_CMD_LOOK           VOCAB4('l','o','o','k')
#define VOCAB_CMD_TRACK          VOCAB4('t','r','a','c')
#define VOCAB_CMD_RUN            VOCAB3('r','u','n')
#define VOCAB_CMD_GET            VOCAB3('g','e','t')
#define VOCAB_CMD_NEXT           VOCAB4('n','e','x','t')
#define VOCAB_CMD_GAZE           VOCAB4('g','a','z','e')
#define VOCAB_CMD_ARM            VOCAB3('a','r','m')
#define VOCAB_CMD_ACK            VOCAB3('a','c','k')
#define VOCAB_CMD_NACK           VOCAB4('n','a','c','k')
#define VOCAB_CMD_MERGE          VOCAB4('m','e','r','g')

using namespace std;
using namespace yarp::os;

class MergePointCloudsManager: public RFModule
{
protected:

    int n_mov;
    Port rpc;
    Port out_explorer;
    Port out_merger;
    Port out_obj_reconstr;

    bool close()
    {
        rpc.close();
        out_explorer.close();
        out_merger.close();
        out_obj_reconstr.close();
        return true;
    }

    bool interruptModule()
    {
        rpc.interrupt();
        out_explorer.interrupt();
        out_merger.interrupt();
        out_obj_reconstr.interrupt();
        return true;
    }

    double getPeriod()
    {
        return 0.1;
    }

    bool respond(const Bottle& cmd, Bottle& reply) 
    {
        if (cmd.get(0).asString()=="merge")
        {
            string object=cmd.get(1).asString();
            Bottle bot1,bot2;
            if (out_obj_reconstr.getOutputCount()>0)
            {
                bot1.clear(); bot2.clear();
                bot1.addString("set");
                bot1.addString("write");
                bot1.addString("on");
                out_obj_reconstr.write(bot1, bot2);
            }

            if (out_explorer.getOutputCount()>0)
            {
                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_HOME);
                bot1.addVocab(VOCAB_CMD_ARM);
                out_explorer.write(bot1,bot2);

                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_BLOCK);
                bot1.addVocab(VOCAB_CMD_GAZE);
                out_explorer.write(bot1,bot2);

                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_LOOK);
                bot1.addString(object.c_str());
                out_explorer.write(bot1,bot2);

                if (bot2.get(0).asVocab()==VOCAB_CMD_NACK)
                {
                    reply.addVocab(VOCAB_CMD_NACK);
                    return true;
                }

                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_TRACK);
                bot1.addVocab(VOCAB_CMD_GAZE);
                bot1.addString("on");
                out_explorer.write(bot1,bot2);

                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_TRACK);
                bot1.addVocab(VOCAB_CMD_ARM);
                bot1.addString("on");
                out_explorer.write(bot1,bot2);

                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_RUN);
                out_explorer.write(bot1,bot2);
            }

            for (int i=0; i<n_mov; i++)
            {
                if (out_explorer.getOutputCount()>0)
                {
                    bot1.clear(); bot2.clear();
                    bot1.addVocab(VOCAB_CMD_NEXT);
                    out_explorer.write(bot1,bot2);

                    bot1.clear(); bot2.clear();
                    bot1.addVocab(VOCAB_CMD_GET);
                    bot1.addString(object.c_str());
                    out_explorer.write(bot1,bot2);

                    int u=0;
                    int v=0;
                
                    if (bot2.get(0).asVocab()==VOCAB_CMD_ACK)
                    {
                        Bottle* point=bot2.get(1).asList();
                        u=point->get(0).asInt();
                        v=point->get(1).asInt();
                    }
                    else
                    {
                        reply.addVocab(VOCAB_CMD_NACK);
                        return true;
                    }

                    if (out_obj_reconstr.getOutputCount()>0)
                    {
                        bot1.clear(); bot2.clear();
                        bot1.addInt(u);
                        bot1.addInt(v);
                        out_obj_reconstr.write(bot1, bot2);

                        bot1.clear(); bot2.clear();
                        bot1.addString("3Drec");
                        out_obj_reconstr.write(bot1, bot2);
                    }
                }
            }

            if (out_merger.getOutputCount())
            {
                bot1.clear(); bot2.clear();
                bot1.addVocab(VOCAB_CMD_MERGE);
                out_merger.write(bot1,bot2);
            }

            reply.addVocab(VOCAB_CMD_ACK);
            return true;
        }
        else
        {
            reply.addString("command not recognized");
            return true;
        }
    }

    bool updateModule()
    {
        return true;
    }

public:
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("manager")).asString().c_str();
        rpc.open(("/"+name+"/rpc").c_str());
        attach(rpc);
        out_explorer.open(("/"+name+"/explorer:o").c_str());
        out_merger.open(("/"+name+"/merger:o").c_str());
        out_obj_reconstr.open(("/"+name+"/object-reconstruction:o").c_str());
        n_mov=rf.check("n_mov",Value(3)).asInt();
        
        return true;
    }
};

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("merge-point-clouds");
    rf.configure(argc,argv);

    MergePointCloudsManager mod;
    mod.runModule(rf);

    return 0;
}

