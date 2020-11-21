.class public Ljavax/jmdns/impl/tasks/state/Announcer;
.super Ljavax/jmdns/impl/tasks/state/DNSStateTask;
.source "Announcer.java"


# static fields
.field static logger:Ljava/util/logging/Logger;


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 25
    const-class v0, Ljavax/jmdns/impl/tasks/state/Announcer;

    invoke-virtual {v0}, Ljava/lang/Class;->getName()Ljava/lang/String;

    move-result-object v0

    invoke-static {v0}, Ljava/util/logging/Logger;->getLogger(Ljava/lang/String;)Ljava/util/logging/Logger;

    move-result-object v0

    sput-object v0, Ljavax/jmdns/impl/tasks/state/Announcer;->logger:Ljava/util/logging/Logger;

    return-void
.end method

.method public constructor <init>(Ljavax/jmdns/impl/JmDNSImpl;)V
    .registers 3
    .param p1, "jmDNSImpl"    # Ljavax/jmdns/impl/JmDNSImpl;

    .line 28
    invoke-static {}, Ljavax/jmdns/impl/tasks/state/Announcer;->defaultTTL()I

    move-result v0

    invoke-direct {p0, p1, v0}, Ljavax/jmdns/impl/tasks/state/DNSStateTask;-><init>(Ljavax/jmdns/impl/JmDNSImpl;I)V

    .line 30
    sget-object v0, Ljavax/jmdns/impl/constants/DNSState;->ANNOUNCING_1:Ljavax/jmdns/impl/constants/DNSState;

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/tasks/state/Announcer;->setTaskState(Ljavax/jmdns/impl/constants/DNSState;)V

    .line 31
    sget-object v0, Ljavax/jmdns/impl/constants/DNSState;->ANNOUNCING_1:Ljavax/jmdns/impl/constants/DNSState;

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/tasks/state/Announcer;->associate(Ljavax/jmdns/impl/constants/DNSState;)V

    .line 32
    return-void
.end method


# virtual methods
.method protected advanceTask()V
    .registers 2

    .line 138
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getTaskState()Ljavax/jmdns/impl/constants/DNSState;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSState;->advance()Ljavax/jmdns/impl/constants/DNSState;

    move-result-object v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/tasks/state/Announcer;->setTaskState(Ljavax/jmdns/impl/constants/DNSState;)V

    .line 139
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getTaskState()Ljavax/jmdns/impl/constants/DNSState;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSState;->isAnnouncing()Z

    move-result v0

    if-nez v0, :cond_1f

    .line 140
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->cancel()Z

    .line 142
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->startRenewer()V

    .line 144
    :cond_1f
    return-void
.end method

.method protected buildOutgoingForDNS(Ljavax/jmdns/impl/DNSOutgoing;)Ljavax/jmdns/impl/DNSOutgoing;
    .registers 6
    .param p1, "out"    # Ljavax/jmdns/impl/DNSOutgoing;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 103
    move-object v0, p1

    .line 104
    .local v0, "newOut":Ljavax/jmdns/impl/DNSOutgoing;
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v1

    invoke-virtual {v1}, Ljavax/jmdns/impl/JmDNSImpl;->getLocalHost()Ljavax/jmdns/impl/HostInfo;

    move-result-object v1

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getTTL()I

    move-result v2

    const/4 v3, 0x1

    invoke-virtual {v1, v3, v2}, Ljavax/jmdns/impl/HostInfo;->answers(ZI)Ljava/util/Collection;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Collection;->iterator()Ljava/util/Iterator;

    move-result-object v1

    :goto_16
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_28

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Ljavax/jmdns/impl/DNSRecord;

    .line 105
    .local v2, "answer":Ljavax/jmdns/impl/DNSRecord;
    const/4 v3, 0x0

    invoke-virtual {p0, v0, v3, v2}, Ljavax/jmdns/impl/tasks/state/Announcer;->addAnswer(Ljavax/jmdns/impl/DNSOutgoing;Ljavax/jmdns/impl/DNSIncoming;Ljavax/jmdns/impl/DNSRecord;)Ljavax/jmdns/impl/DNSOutgoing;

    move-result-object v0

    .line 106
    .end local v2    # "answer":Ljavax/jmdns/impl/DNSRecord;
    goto :goto_16

    .line 107
    :cond_28
    return-object v0
.end method

.method protected buildOutgoingForInfo(Ljavax/jmdns/impl/ServiceInfoImpl;Ljavax/jmdns/impl/DNSOutgoing;)Ljavax/jmdns/impl/DNSOutgoing;
    .registers 7
    .param p1, "info"    # Ljavax/jmdns/impl/ServiceInfoImpl;
    .param p2, "out"    # Ljavax/jmdns/impl/DNSOutgoing;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 116
    move-object v0, p2

    .line 117
    .local v0, "newOut":Ljavax/jmdns/impl/DNSOutgoing;
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getTTL()I

    move-result v1

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v2

    invoke-virtual {v2}, Ljavax/jmdns/impl/JmDNSImpl;->getLocalHost()Ljavax/jmdns/impl/HostInfo;

    move-result-object v2

    const/4 v3, 0x1

    invoke-virtual {p1, v3, v1, v2}, Ljavax/jmdns/impl/ServiceInfoImpl;->answers(ZILjavax/jmdns/impl/HostInfo;)Ljava/util/Collection;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Collection;->iterator()Ljava/util/Iterator;

    move-result-object v1

    :goto_16
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_28

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Ljavax/jmdns/impl/DNSRecord;

    .line 118
    .local v2, "answer":Ljavax/jmdns/impl/DNSRecord;
    const/4 v3, 0x0

    invoke-virtual {p0, v0, v3, v2}, Ljavax/jmdns/impl/tasks/state/Announcer;->addAnswer(Ljavax/jmdns/impl/DNSOutgoing;Ljavax/jmdns/impl/DNSIncoming;Ljavax/jmdns/impl/DNSRecord;)Ljavax/jmdns/impl/DNSOutgoing;

    move-result-object v0

    .line 119
    .end local v2    # "answer":Ljavax/jmdns/impl/DNSRecord;
    goto :goto_16

    .line 120
    :cond_28
    return-object v0
.end method

.method public cancel()Z
    .registers 2

    .line 65
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->removeAssociation()V

    .line 67
    invoke-super {p0}, Ljavax/jmdns/impl/tasks/state/DNSStateTask;->cancel()Z

    move-result v0

    return v0
.end method

.method protected checkRunCondition()Z
    .registers 2

    .line 85
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->isCanceling()Z

    move-result v0

    if-nez v0, :cond_16

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->isCanceled()Z

    move-result v0

    if-nez v0, :cond_16

    const/4 v0, 0x1

    goto :goto_17

    :cond_16
    const/4 v0, 0x0

    :goto_17
    return v0
.end method

.method protected createOugoing()Ljavax/jmdns/impl/DNSOutgoing;
    .registers 3

    .line 94
    new-instance v0, Ljavax/jmdns/impl/DNSOutgoing;

    const v1, 0x8400

    invoke-direct {v0, v1}, Ljavax/jmdns/impl/DNSOutgoing;-><init>(I)V

    return-object v0
.end method

.method public getName()Ljava/lang/String;
    .registers 3

    .line 40
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    const-string v1, "Announcer("

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v1

    if-eqz v1, :cond_19

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v1

    invoke-virtual {v1}, Ljavax/jmdns/impl/JmDNSImpl;->getName()Ljava/lang/String;

    move-result-object v1

    goto :goto_1b

    :cond_19
    const-string v1, ""

    :goto_1b
    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, ")"

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getTaskDescription()Ljava/lang/String;
    .registers 2

    .line 76
    const-string v0, "announcing"

    return-object v0
.end method

.method protected recoverTask(Ljava/lang/Throwable;)V
    .registers 3
    .param p1, "e"    # Ljava/lang/Throwable;

    .line 129
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->recover()V

    .line 130
    return-void
.end method

.method public start(Ljava/util/Timer;)V
    .registers 9
    .param p1, "timer"    # Ljava/util/Timer;

    .line 58
    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->isCanceling()Z

    move-result v0

    if-nez v0, :cond_1d

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->isCanceled()Z

    move-result v0

    if-nez v0, :cond_1d

    .line 59
    const-wide/16 v3, 0x3e8

    const-wide/16 v5, 0x3e8

    move-object v1, p1

    move-object v2, p0

    invoke-virtual/range {v1 .. v6}, Ljava/util/Timer;->schedule(Ljava/util/TimerTask;JJ)V

    .line 61
    :cond_1d
    return-void
.end method

.method public toString()Ljava/lang/String;
    .registers 3

    .line 49
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    invoke-super {p0}, Ljavax/jmdns/impl/tasks/state/DNSStateTask;->toString()Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, " state: "

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {p0}, Ljavax/jmdns/impl/tasks/state/Announcer;->getTaskState()Ljavax/jmdns/impl/constants/DNSState;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/Object;)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method
