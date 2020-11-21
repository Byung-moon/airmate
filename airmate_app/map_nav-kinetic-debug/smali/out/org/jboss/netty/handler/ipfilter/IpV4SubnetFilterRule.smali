.class public Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;
.super Lorg/jboss/netty/handler/ipfilter/IpV4Subnet;
.source "IpV4SubnetFilterRule.java"

# interfaces
.implements Lorg/jboss/netty/handler/ipfilter/IpFilterRule;


# instance fields
.field private isAllowRule:Z


# direct methods
.method public constructor <init>(Z)V
    .registers 3
    .param p1, "allow"    # Z

    .line 31
    invoke-direct {p0}, Lorg/jboss/netty/handler/ipfilter/IpV4Subnet;-><init>()V

    .line 24
    const/4 v0, 0x1

    iput-boolean v0, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 32
    iput-boolean p1, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 33
    return-void
.end method

.method public constructor <init>(ZLjava/lang/String;)V
    .registers 4
    .param p1, "allow"    # Z
    .param p2, "netAddress"    # Ljava/lang/String;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/net/UnknownHostException;
        }
    .end annotation

    .line 49
    invoke-direct {p0, p2}, Lorg/jboss/netty/handler/ipfilter/IpV4Subnet;-><init>(Ljava/lang/String;)V

    .line 24
    const/4 v0, 0x1

    iput-boolean v0, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 50
    iput-boolean p1, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 51
    return-void
.end method

.method public constructor <init>(ZLjava/net/InetAddress;I)V
    .registers 5
    .param p1, "allow"    # Z
    .param p2, "inetAddress"    # Ljava/net/InetAddress;
    .param p3, "cidrNetMask"    # I

    .line 37
    invoke-direct {p0, p2, p3}, Lorg/jboss/netty/handler/ipfilter/IpV4Subnet;-><init>(Ljava/net/InetAddress;I)V

    .line 24
    const/4 v0, 0x1

    iput-boolean v0, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 38
    iput-boolean p1, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 39
    return-void
.end method

.method public constructor <init>(ZLjava/net/InetAddress;Ljava/lang/String;)V
    .registers 5
    .param p1, "allow"    # Z
    .param p2, "inetAddress"    # Ljava/net/InetAddress;
    .param p3, "netMask"    # Ljava/lang/String;

    .line 43
    invoke-direct {p0, p2, p3}, Lorg/jboss/netty/handler/ipfilter/IpV4Subnet;-><init>(Ljava/net/InetAddress;Ljava/lang/String;)V

    .line 24
    const/4 v0, 0x1

    iput-boolean v0, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 44
    iput-boolean p1, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    .line 45
    return-void
.end method


# virtual methods
.method public isAllowRule()Z
    .registers 2

    .line 54
    iget-boolean v0, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    return v0
.end method

.method public isDenyRule()Z
    .registers 2

    .line 58
    iget-boolean v0, p0, Lorg/jboss/netty/handler/ipfilter/IpV4SubnetFilterRule;->isAllowRule:Z

    xor-int/lit8 v0, v0, 0x1

    return v0
.end method
