.class public Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;
.super Ljava/util/AbstractMap;
.source "JmDNSImpl.java"

# interfaces
.implements Ljava/lang/Cloneable;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Ljavax/jmdns/impl/JmDNSImpl;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x9
    name = "ServiceTypeEntry"
.end annotation

.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry$SubTypeEntry;
    }
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/util/AbstractMap<",
        "Ljava/lang/String;",
        "Ljava/lang/String;",
        ">;",
        "Ljava/lang/Cloneable;"
    }
.end annotation


# instance fields
.field private final _entrySet:Ljava/util/Set;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "Ljava/lang/String;",
            "Ljava/lang/String;",
            ">;>;"
        }
    .end annotation
.end field

.field private final _type:Ljava/lang/String;


# direct methods
.method public constructor <init>(Ljava/lang/String;)V
    .registers 3
    .param p1, "type"    # Ljava/lang/String;

    .line 202
    invoke-direct {p0}, Ljava/util/AbstractMap;-><init>()V

    .line 203
    iput-object p1, p0, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->_type:Ljava/lang/String;

    .line 204
    new-instance v0, Ljava/util/HashSet;

    invoke-direct {v0}, Ljava/util/HashSet;-><init>()V

    iput-object v0, p0, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->_entrySet:Ljava/util/Set;

    .line 205
    return-void
.end method


# virtual methods
.method public add(Ljava/lang/String;)Z
    .registers 4
    .param p1, "subtype"    # Ljava/lang/String;

    .line 246
    if-eqz p1, :cond_15

    invoke-virtual {p0, p1}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->contains(Ljava/lang/String;)Z

    move-result v0

    if-eqz v0, :cond_9

    goto :goto_15

    .line 249
    :cond_9
    iget-object v0, p0, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->_entrySet:Ljava/util/Set;

    new-instance v1, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry$SubTypeEntry;

    invoke-direct {v1, p1}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry$SubTypeEntry;-><init>(Ljava/lang/String;)V

    invoke-interface {v0, v1}, Ljava/util/Set;->add(Ljava/lang/Object;)Z

    .line 250
    const/4 v0, 0x1

    return v0

    .line 247
    :cond_15
    :goto_15
    const/4 v0, 0x0

    return v0
.end method

.method public bridge synthetic clone()Ljava/lang/Object;
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/CloneNotSupportedException;
        }
    .end annotation

    .line 113
    invoke-virtual {p0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->clone()Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;

    move-result-object v0

    return-object v0
.end method

.method public clone()Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;
    .registers 5

    .line 268
    new-instance v0, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;

    invoke-virtual {p0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->getType()Ljava/lang/String;

    move-result-object v1

    invoke-direct {v0, v1}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;-><init>(Ljava/lang/String;)V

    .line 269
    .local v0, "entry":Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;
    invoke-virtual {p0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->entrySet()Ljava/util/Set;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Set;->iterator()Ljava/util/Iterator;

    move-result-object v1

    :goto_11
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_27

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Ljava/util/Map$Entry;

    .line 270
    .local v2, "subTypeEntry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<Ljava/lang/String;Ljava/lang/String;>;"
    invoke-interface {v2}, Ljava/util/Map$Entry;->getValue()Ljava/lang/Object;

    move-result-object v3

    check-cast v3, Ljava/lang/String;

    invoke-virtual {v0, v3}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->add(Ljava/lang/String;)Z

    .line 271
    .end local v2    # "subTypeEntry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<Ljava/lang/String;Ljava/lang/String;>;"
    goto :goto_11

    .line 272
    :cond_27
    return-object v0
.end method

.method public contains(Ljava/lang/String;)Z
    .registers 3
    .param p1, "subtype"    # Ljava/lang/String;

    .line 234
    if-eqz p1, :cond_e

    invoke-virtual {p1}, Ljava/lang/String;->toLowerCase()Ljava/lang/String;

    move-result-object v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->containsKey(Ljava/lang/Object;)Z

    move-result v0

    if-eqz v0, :cond_e

    const/4 v0, 0x1

    goto :goto_f

    :cond_e
    const/4 v0, 0x0

    :goto_f
    return v0
.end method

.method public entrySet()Ljava/util/Set;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "Ljava/lang/String;",
            "Ljava/lang/String;",
            ">;>;"
        }
    .end annotation

    .line 222
    iget-object v0, p0, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->_entrySet:Ljava/util/Set;

    return-object v0
.end method

.method public getType()Ljava/lang/String;
    .registers 2

    .line 213
    iget-object v0, p0, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->_type:Ljava/lang/String;

    return-object v0
.end method

.method public iterator()Ljava/util/Iterator;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Iterator<",
            "Ljava/lang/String;",
            ">;"
        }
    .end annotation

    .line 259
    invoke-virtual {p0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->keySet()Ljava/util/Set;

    move-result-object v0

    invoke-interface {v0}, Ljava/util/Set;->iterator()Ljava/util/Iterator;

    move-result-object v0

    return-object v0
.end method

.method public toString()Ljava/lang/String;
    .registers 5

    .line 281
    new-instance v0, Ljava/lang/StringBuilder;

    const/16 v1, 0xc8

    invoke-direct {v0, v1}, Ljava/lang/StringBuilder;-><init>(I)V

    .line 282
    .local v0, "aLog":Ljava/lang/StringBuilder;
    invoke-virtual {p0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->isEmpty()Z

    move-result v1

    if-eqz v1, :cond_13

    .line 283
    const-string v1, "empty"

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    goto :goto_39

    .line 285
    :cond_13
    invoke-virtual {p0}, Ljavax/jmdns/impl/JmDNSImpl$ServiceTypeEntry;->values()Ljava/util/Collection;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Collection;->iterator()Ljava/util/Iterator;

    move-result-object v1

    :goto_1b
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_30

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Ljava/lang/String;

    .line 286
    .local v2, "value":Ljava/lang/String;
    invoke-virtual {v0, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 287
    const-string v3, ", "

    invoke-virtual {v0, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 288
    .end local v2    # "value":Ljava/lang/String;
    goto :goto_1b

    .line 289
    :cond_30
    invoke-virtual {v0}, Ljava/lang/StringBuilder;->length()I

    move-result v1

    add-int/lit8 v1, v1, -0x2

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->setLength(I)V

    .line 291
    :goto_39
    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v1

    return-object v1
.end method
