.class final Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;
.super Ljavax/jmdns/impl/DNSStatefulObject$DefaultImplementation;
.source "ServiceInfoImpl.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Ljavax/jmdns/impl/ServiceInfoImpl;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1a
    name = "ServiceInfoState"
.end annotation


# static fields
.field private static final serialVersionUID:J = 0xf52a9538bcf62d4L


# instance fields
.field private final _info:Ljavax/jmdns/impl/ServiceInfoImpl;


# direct methods
.method public constructor <init>(Ljavax/jmdns/impl/ServiceInfoImpl;)V
    .registers 2
    .param p1, "info"    # Ljavax/jmdns/impl/ServiceInfoImpl;

    .line 84
    invoke-direct {p0}, Ljavax/jmdns/impl/DNSStatefulObject$DefaultImplementation;-><init>()V

    .line 85
    iput-object p1, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_info:Ljavax/jmdns/impl/ServiceInfoImpl;

    .line 86
    return-void
.end method


# virtual methods
.method public setDns(Ljavax/jmdns/impl/JmDNSImpl;)V
    .registers 2
    .param p1, "dns"    # Ljavax/jmdns/impl/JmDNSImpl;

    .line 111
    invoke-super {p0, p1}, Ljavax/jmdns/impl/DNSStatefulObject$DefaultImplementation;->setDns(Ljavax/jmdns/impl/JmDNSImpl;)V

    .line 112
    return-void
.end method

.method protected setTask(Ljavax/jmdns/impl/tasks/DNSTask;)V
    .registers 4
    .param p1, "task"    # Ljavax/jmdns/impl/tasks/DNSTask;

    .line 90
    invoke-super {p0, p1}, Ljavax/jmdns/impl/DNSStatefulObject$DefaultImplementation;->setTask(Ljavax/jmdns/impl/tasks/DNSTask;)V

    .line 91
    iget-object v0, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_task:Ljavax/jmdns/impl/tasks/DNSTask;

    if-nez v0, :cond_47

    iget-object v0, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_info:Ljavax/jmdns/impl/ServiceInfoImpl;

    invoke-virtual {v0}, Ljavax/jmdns/impl/ServiceInfoImpl;->needTextAnnouncing()Z

    move-result v0

    if-eqz v0, :cond_47

    .line 92
    invoke-virtual {p0}, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->lock()V

    .line 94
    :try_start_12
    iget-object v0, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_task:Ljavax/jmdns/impl/tasks/DNSTask;

    if-nez v0, :cond_3e

    iget-object v0, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_info:Ljavax/jmdns/impl/ServiceInfoImpl;

    invoke-virtual {v0}, Ljavax/jmdns/impl/ServiceInfoImpl;->needTextAnnouncing()Z

    move-result v0

    if-eqz v0, :cond_3e

    .line 95
    iget-object v0, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_state:Ljavax/jmdns/impl/constants/DNSState;

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSState;->isAnnounced()Z

    move-result v0

    if-eqz v0, :cond_38

    .line 96
    sget-object v0, Ljavax/jmdns/impl/constants/DNSState;->ANNOUNCING_1:Ljavax/jmdns/impl/constants/DNSState;

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->setState(Ljavax/jmdns/impl/constants/DNSState;)V

    .line 97
    invoke-virtual {p0}, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    if-eqz v0, :cond_38

    .line 98
    invoke-virtual {p0}, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->getDns()Ljavax/jmdns/impl/JmDNSImpl;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/JmDNSImpl;->startAnnouncer()V

    .line 101
    :cond_38
    iget-object v0, p0, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->_info:Ljavax/jmdns/impl/ServiceInfoImpl;

    const/4 v1, 0x0

    invoke-virtual {v0, v1}, Ljavax/jmdns/impl/ServiceInfoImpl;->setNeedTextAnnouncing(Z)V
    :try_end_3e
    .catchall {:try_start_12 .. :try_end_3e} :catchall_42

    .line 104
    :cond_3e
    invoke-virtual {p0}, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->unlock()V

    .line 105
    goto :goto_47

    .line 104
    :catchall_42
    move-exception v0

    invoke-virtual {p0}, Ljavax/jmdns/impl/ServiceInfoImpl$ServiceInfoState;->unlock()V

    throw v0

    .line 107
    :cond_47
    :goto_47
    return-void
.end method
