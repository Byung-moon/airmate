.class final Lcom/google/common/collect/RegularImmutableMap$TerminalEntry;
.super Lcom/google/common/collect/ImmutableEntry;
.source "RegularImmutableMap.java"

# interfaces
.implements Lcom/google/common/collect/RegularImmutableMap$LinkedEntry;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/RegularImmutableMap;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1a
    name = "TerminalEntry"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<K:",
        "Ljava/lang/Object;",
        "V:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/ImmutableEntry<",
        "TK;TV;>;",
        "Lcom/google/common/collect/RegularImmutableMap$LinkedEntry<",
        "TK;TV;>;"
    }
.end annotation

.annotation build Ljavax/annotation/concurrent/Immutable;
.end annotation


# direct methods
.method constructor <init>(Ljava/lang/Object;Ljava/lang/Object;)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TK;TV;)V"
        }
    .end annotation

    .line 132
    .local p0, "this":Lcom/google/common/collect/RegularImmutableMap$TerminalEntry;, "Lcom/google/common/collect/RegularImmutableMap$TerminalEntry<TK;TV;>;"
    .local p1, "key":Ljava/lang/Object;, "TK;"
    .local p2, "value":Ljava/lang/Object;, "TV;"
    invoke-direct {p0, p1, p2}, Lcom/google/common/collect/ImmutableEntry;-><init>(Ljava/lang/Object;Ljava/lang/Object;)V

    .line 133
    return-void
.end method


# virtual methods
.method public next()Lcom/google/common/collect/RegularImmutableMap$LinkedEntry;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Lcom/google/common/collect/RegularImmutableMap$LinkedEntry<",
            "TK;TV;>;"
        }
    .end annotation

    .annotation runtime Ljavax/annotation/Nullable;
    .end annotation

    .line 136
    .local p0, "this":Lcom/google/common/collect/RegularImmutableMap$TerminalEntry;, "Lcom/google/common/collect/RegularImmutableMap$TerminalEntry<TK;TV;>;"
    const/4 v0, 0x0

    return-object v0
.end method
