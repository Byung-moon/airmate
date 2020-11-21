.class public abstract Lorg/apache/commons/httpclient/cookie/CookiePolicy;
.super Ljava/lang/Object;
.source "CookiePolicy.java"


# static fields
.field public static final BROWSER_COMPATIBILITY:Ljava/lang/String; = "compatibility"

.field public static final COMPATIBILITY:I = 0x0

.field public static final DEFAULT:Ljava/lang/String; = "default"

.field public static final IGNORE_COOKIES:Ljava/lang/String; = "ignoreCookies"

.field protected static final LOG:Lorg/apache/commons/logging/Log;

.field public static final NETSCAPE:Ljava/lang/String; = "netscape"

.field public static final NETSCAPE_DRAFT:I = 0x1

.field public static final RFC2109:I = 0x2

.field public static final RFC2965:I = 0x3

.field public static final RFC_2109:Ljava/lang/String; = "rfc2109"

.field public static final RFC_2965:Ljava/lang/String; = "rfc2965"

.field private static SPECS:Ljava/util/Map;

.field static synthetic class$org$apache$commons$httpclient$cookie$CookiePolicy:Ljava/lang/Class;

.field static synthetic class$org$apache$commons$httpclient$cookie$CookieSpecBase:Ljava/lang/Class;

.field static synthetic class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec:Ljava/lang/Class;

.field static synthetic class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec:Ljava/lang/Class;

.field static synthetic class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

.field static synthetic class$org$apache$commons$httpclient$cookie$RFC2965Spec:Ljava/lang/Class;

.field private static defaultPolicy:I


# direct methods
.method static constructor <clinit>()V
    .registers 2

    .line 61
    new-instance v0, Ljava/util/HashMap;

    invoke-direct {v0}, Ljava/util/HashMap;-><init>()V

    invoke-static {v0}, Ljava/util/Collections;->synchronizedMap(Ljava/util/Map;)Ljava/util/Map;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->SPECS:Ljava/util/Map;

    .line 107
    const-string v0, "default"

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

    if-nez v1, :cond_1a

    const-string v1, "org.apache.commons.httpclient.cookie.RFC2109Spec"

    invoke-static {v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v1

    sput-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

    goto :goto_1c

    :cond_1a
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

    :goto_1c
    invoke-static {v0, v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V

    .line 108
    const-string v0, "rfc2109"

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

    if-nez v1, :cond_2e

    const-string v1, "org.apache.commons.httpclient.cookie.RFC2109Spec"

    invoke-static {v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v1

    sput-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

    goto :goto_30

    :cond_2e
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2109Spec:Ljava/lang/Class;

    :goto_30
    invoke-static {v0, v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V

    .line 109
    const-string v0, "rfc2965"

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2965Spec:Ljava/lang/Class;

    if-nez v1, :cond_42

    const-string v1, "org.apache.commons.httpclient.cookie.RFC2965Spec"

    invoke-static {v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v1

    sput-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2965Spec:Ljava/lang/Class;

    goto :goto_44

    :cond_42
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$RFC2965Spec:Ljava/lang/Class;

    :goto_44
    invoke-static {v0, v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V

    .line 110
    const-string v0, "compatibility"

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$CookieSpecBase:Ljava/lang/Class;

    if-nez v1, :cond_56

    const-string v1, "org.apache.commons.httpclient.cookie.CookieSpecBase"

    invoke-static {v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v1

    sput-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$CookieSpecBase:Ljava/lang/Class;

    goto :goto_58

    :cond_56
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$CookieSpecBase:Ljava/lang/Class;

    :goto_58
    invoke-static {v0, v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V

    .line 111
    const-string v0, "netscape"

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec:Ljava/lang/Class;

    if-nez v1, :cond_6a

    const-string v1, "org.apache.commons.httpclient.cookie.NetscapeDraftSpec"

    invoke-static {v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v1

    sput-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec:Ljava/lang/Class;

    goto :goto_6c

    :cond_6a
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec:Ljava/lang/Class;

    :goto_6c
    invoke-static {v0, v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V

    .line 112
    const-string v0, "ignoreCookies"

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec:Ljava/lang/Class;

    if-nez v1, :cond_7e

    const-string v1, "org.apache.commons.httpclient.cookie.IgnoreCookiesSpec"

    invoke-static {v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v1

    sput-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec:Ljava/lang/Class;

    goto :goto_80

    :cond_7e
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec:Ljava/lang/Class;

    :goto_80
    invoke-static {v0, v1}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V

    .line 149
    const/4 v0, 0x2

    sput v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->defaultPolicy:I

    .line 152
    sget-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$CookiePolicy:Ljava/lang/Class;

    if-nez v0, :cond_93

    const-string v0, "org.apache.commons.httpclient.cookie.CookiePolicy"

    invoke-static {v0}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$CookiePolicy:Ljava/lang/Class;

    goto :goto_95

    :cond_93
    sget-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->class$org$apache$commons$httpclient$cookie$CookiePolicy:Ljava/lang/Class;

    :goto_95
    invoke-static {v0}, Lorg/apache/commons/logging/LogFactory;->getLog(Ljava/lang/Class;)Lorg/apache/commons/logging/Log;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->LOG:Lorg/apache/commons/logging/Log;

    return-void
.end method

.method public constructor <init>()V
    .registers 1

    .line 59
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method

.method static synthetic class$(Ljava/lang/String;)Ljava/lang/Class;
    .registers 4
    .param p0, "x0"    # Ljava/lang/String;

    .line 107
    :try_start_0
    invoke-static {p0}, Ljava/lang/Class;->forName(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0
    :try_end_4
    .catch Ljava/lang/ClassNotFoundException; {:try_start_0 .. :try_end_4} :catch_5

    return-object v0

    :catch_5
    move-exception v0

    .local v0, "x1":Ljava/lang/ClassNotFoundException;
    new-instance v1, Ljava/lang/NoClassDefFoundError;

    invoke-virtual {v0}, Ljava/lang/ClassNotFoundException;->getMessage()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v1, v2}, Ljava/lang/NoClassDefFoundError;-><init>(Ljava/lang/String;)V

    throw v1
.end method

.method public static getCompatibilitySpec()Lorg/apache/commons/httpclient/cookie/CookieSpec;
    .registers 1

    .line 321
    const/4 v0, 0x0

    invoke-static {v0}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->getSpecByPolicy(I)Lorg/apache/commons/httpclient/cookie/CookieSpec;

    move-result-object v0

    return-object v0
.end method

.method public static getCookieSpec(Ljava/lang/String;)Lorg/apache/commons/httpclient/cookie/CookieSpec;
    .registers 6
    .param p0, "id"    # Ljava/lang/String;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/IllegalStateException;
        }
    .end annotation

    .line 205
    if-eqz p0, :cond_68

    .line 208
    sget-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->SPECS:Ljava/util/Map;

    invoke-virtual {p0}, Ljava/lang/String;->toLowerCase()Ljava/lang/String;

    move-result-object v1

    invoke-interface {v0, v1}, Ljava/util/Map;->get(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/Class;

    .line 210
    .local v0, "clazz":Ljava/lang/Class;
    if-eqz v0, :cond_51

    .line 212
    :try_start_10
    invoke-virtual {v0}, Ljava/lang/Class;->newInstance()Ljava/lang/Object;

    move-result-object v1

    check-cast v1, Lorg/apache/commons/httpclient/cookie/CookieSpec;
    :try_end_16
    .catch Ljava/lang/Exception; {:try_start_10 .. :try_end_16} :catch_17

    return-object v1

    .line 213
    :catch_17
    move-exception v1

    .line 214
    .local v1, "e":Ljava/lang/Exception;
    sget-object v2, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->LOG:Lorg/apache/commons/logging/Log;

    new-instance v3, Ljava/lang/StringBuffer;

    invoke-direct {v3}, Ljava/lang/StringBuffer;-><init>()V

    const-string v4, "Error initializing cookie spec: "

    invoke-virtual {v3, v4}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v3, p0}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v3}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v3

    invoke-interface {v2, v3, v1}, Lorg/apache/commons/logging/Log;->error(Ljava/lang/Object;Ljava/lang/Throwable;)V

    .line 215
    new-instance v2, Ljava/lang/IllegalStateException;

    new-instance v3, Ljava/lang/StringBuffer;

    invoke-direct {v3}, Ljava/lang/StringBuffer;-><init>()V

    invoke-virtual {v3, p0}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    const-string v4, " cookie spec implemented by "

    invoke-virtual {v3, v4}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v0}, Ljava/lang/Class;->getName()Ljava/lang/String;

    move-result-object v4

    invoke-virtual {v3, v4}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    const-string v4, " could not be initialized"

    invoke-virtual {v3, v4}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v3}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v3

    invoke-direct {v2, v3}, Ljava/lang/IllegalStateException;-><init>(Ljava/lang/String;)V

    throw v2

    .line 220
    .end local v1    # "e":Ljava/lang/Exception;
    :cond_51
    new-instance v1, Ljava/lang/IllegalStateException;

    new-instance v2, Ljava/lang/StringBuffer;

    invoke-direct {v2}, Ljava/lang/StringBuffer;-><init>()V

    const-string v3, "Unsupported cookie spec "

    invoke-virtual {v2, v3}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v2, p0}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v2}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v1, v2}, Ljava/lang/IllegalStateException;-><init>(Ljava/lang/String;)V

    throw v1

    .line 206
    .end local v0    # "clazz":Ljava/lang/Class;
    :cond_68
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "Id may not be null"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public static getDefaultPolicy()I
    .registers 1

    .line 232
    sget v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->defaultPolicy:I

    return v0
.end method

.method public static getDefaultSpec()Lorg/apache/commons/httpclient/cookie/CookieSpec;
    .registers 3

    .line 279
    :try_start_0
    const-string v0, "default"

    invoke-static {v0}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->getCookieSpec(Ljava/lang/String;)Lorg/apache/commons/httpclient/cookie/CookieSpec;

    move-result-object v0
    :try_end_6
    .catch Ljava/lang/IllegalStateException; {:try_start_0 .. :try_end_6} :catch_7

    return-object v0

    .line 280
    :catch_7
    move-exception v0

    .line 281
    .local v0, "e":Ljava/lang/IllegalStateException;
    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->LOG:Lorg/apache/commons/logging/Log;

    const-string v2, "Default cookie policy is not registered"

    invoke-interface {v1, v2}, Lorg/apache/commons/logging/Log;->warn(Ljava/lang/Object;)V

    .line 282
    new-instance v1, Lorg/apache/commons/httpclient/cookie/RFC2109Spec;

    invoke-direct {v1}, Lorg/apache/commons/httpclient/cookie/RFC2109Spec;-><init>()V

    return-object v1
.end method

.method public static getRegisteredCookieSpecs()[Ljava/lang/String;
    .registers 2

    .line 335
    sget-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->SPECS:Ljava/util/Map;

    invoke-interface {v0}, Ljava/util/Map;->keySet()Ljava/util/Set;

    move-result-object v0

    sget-object v1, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->SPECS:Ljava/util/Map;

    invoke-interface {v1}, Ljava/util/Map;->size()I

    move-result v1

    new-array v1, v1, [Ljava/lang/String;

    invoke-interface {v0, v1}, Ljava/util/Set;->toArray([Ljava/lang/Object;)[Ljava/lang/Object;

    move-result-object v0

    check-cast v0, [Ljava/lang/String;

    return-object v0
.end method

.method public static getSpecByPolicy(I)Lorg/apache/commons/httpclient/cookie/CookieSpec;
    .registers 2
    .param p0, "policy"    # I

    .line 253
    packed-switch p0, :pswitch_data_20

    .line 263
    invoke-static {}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->getDefaultSpec()Lorg/apache/commons/httpclient/cookie/CookieSpec;

    move-result-object v0

    return-object v0

    .line 261
    :pswitch_8
    new-instance v0, Lorg/apache/commons/httpclient/cookie/RFC2965Spec;

    invoke-direct {v0}, Lorg/apache/commons/httpclient/cookie/RFC2965Spec;-><init>()V

    return-object v0

    .line 259
    :pswitch_e
    new-instance v0, Lorg/apache/commons/httpclient/cookie/RFC2109Spec;

    invoke-direct {v0}, Lorg/apache/commons/httpclient/cookie/RFC2109Spec;-><init>()V

    return-object v0

    .line 257
    :pswitch_14
    new-instance v0, Lorg/apache/commons/httpclient/cookie/NetscapeDraftSpec;

    invoke-direct {v0}, Lorg/apache/commons/httpclient/cookie/NetscapeDraftSpec;-><init>()V

    return-object v0

    .line 255
    :pswitch_1a
    new-instance v0, Lorg/apache/commons/httpclient/cookie/CookieSpecBase;

    invoke-direct {v0}, Lorg/apache/commons/httpclient/cookie/CookieSpecBase;-><init>()V

    return-object v0

    :pswitch_data_20
    .packed-switch 0x0
        :pswitch_1a
        :pswitch_14
        :pswitch_e
        :pswitch_8
    .end packed-switch
.end method

.method public static getSpecByVersion(I)Lorg/apache/commons/httpclient/cookie/CookieSpec;
    .registers 2
    .param p0, "ver"    # I

    .line 304
    packed-switch p0, :pswitch_data_14

    .line 310
    invoke-static {}, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->getDefaultSpec()Lorg/apache/commons/httpclient/cookie/CookieSpec;

    move-result-object v0

    return-object v0

    .line 308
    :pswitch_8
    new-instance v0, Lorg/apache/commons/httpclient/cookie/RFC2109Spec;

    invoke-direct {v0}, Lorg/apache/commons/httpclient/cookie/RFC2109Spec;-><init>()V

    return-object v0

    .line 306
    :pswitch_e
    new-instance v0, Lorg/apache/commons/httpclient/cookie/NetscapeDraftSpec;

    invoke-direct {v0}, Lorg/apache/commons/httpclient/cookie/NetscapeDraftSpec;-><init>()V

    return-object v0

    :pswitch_data_14
    .packed-switch 0x0
        :pswitch_e
        :pswitch_8
    .end packed-switch
.end method

.method public static registerCookieSpec(Ljava/lang/String;Ljava/lang/Class;)V
    .registers 4
    .param p0, "id"    # Ljava/lang/String;
    .param p1, "clazz"    # Ljava/lang/Class;

    .line 168
    if-eqz p0, :cond_16

    .line 171
    if-eqz p1, :cond_e

    .line 174
    sget-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->SPECS:Ljava/util/Map;

    invoke-virtual {p0}, Ljava/lang/String;->toLowerCase()Ljava/lang/String;

    move-result-object v1

    invoke-interface {v0, v1, p1}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 175
    return-void

    .line 172
    :cond_e
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "Cookie spec class may not be null"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 169
    :cond_16
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "Id may not be null"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public static setDefaultPolicy(I)V
    .registers 1
    .param p0, "policy"    # I

    .line 243
    sput p0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->defaultPolicy:I

    .line 244
    return-void
.end method

.method public static unregisterCookieSpec(Ljava/lang/String;)V
    .registers 3
    .param p0, "id"    # Ljava/lang/String;

    .line 185
    if-eqz p0, :cond_c

    .line 188
    sget-object v0, Lorg/apache/commons/httpclient/cookie/CookiePolicy;->SPECS:Ljava/util/Map;

    invoke-virtual {p0}, Ljava/lang/String;->toLowerCase()Ljava/lang/String;

    move-result-object v1

    invoke-interface {v0, v1}, Ljava/util/Map;->remove(Ljava/lang/Object;)Ljava/lang/Object;

    .line 189
    return-void

    .line 186
    :cond_c
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "Id may not be null"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method
