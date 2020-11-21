.class public final Lorg/xbill/DNS/Type;
.super Ljava/lang/Object;
.source "Type.java"


# annotations
.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Lorg/xbill/DNS/Type$TypeMnemonic;
    }
.end annotation


# static fields
.field public static final A:I = 0x1

.field public static final A6:I = 0x26

.field public static final AAAA:I = 0x1c

.field public static final AFSDB:I = 0x12

.field public static final ANY:I = 0xff

.field public static final APL:I = 0x2a

.field public static final ATMA:I = 0x22

.field public static final AXFR:I = 0xfc

.field public static final CERT:I = 0x25

.field public static final CNAME:I = 0x5

.field public static final DHCID:I = 0x31

.field public static final DLV:I = 0x8001

.field public static final DNAME:I = 0x27

.field public static final DNSKEY:I = 0x30

.field public static final DS:I = 0x2b

.field public static final EID:I = 0x1f

.field public static final GPOS:I = 0x1b

.field public static final HINFO:I = 0xd

.field public static final IPSECKEY:I = 0x2d

.field public static final ISDN:I = 0x14

.field public static final IXFR:I = 0xfb

.field public static final KEY:I = 0x19

.field public static final KX:I = 0x24

.field public static final LOC:I = 0x1d

.field public static final MAILA:I = 0xfe

.field public static final MAILB:I = 0xfd

.field public static final MB:I = 0x7

.field public static final MD:I = 0x3

.field public static final MF:I = 0x4

.field public static final MG:I = 0x8

.field public static final MINFO:I = 0xe

.field public static final MR:I = 0x9

.field public static final MX:I = 0xf

.field public static final NAPTR:I = 0x23

.field public static final NIMLOC:I = 0x20

.field public static final NS:I = 0x2

.field public static final NSAP:I = 0x16

.field public static final NSAP_PTR:I = 0x17

.field public static final NSEC:I = 0x2f

.field public static final NSEC3:I = 0x32

.field public static final NSEC3PARAM:I = 0x33

.field public static final NULL:I = 0xa

.field public static final NXT:I = 0x1e

.field public static final OPT:I = 0x29

.field public static final PTR:I = 0xc

.field public static final PX:I = 0x1a

.field public static final RP:I = 0x11

.field public static final RRSIG:I = 0x2e

.field public static final RT:I = 0x15

.field public static final SIG:I = 0x18

.field public static final SOA:I = 0x6

.field public static final SPF:I = 0x63

.field public static final SRV:I = 0x21

.field public static final SSHFP:I = 0x2c

.field public static final TKEY:I = 0xf9

.field public static final TSIG:I = 0xfa

.field public static final TXT:I = 0x10

.field public static final WKS:I = 0xb

.field public static final X25:I = 0x13

.field private static types:Lorg/xbill/DNS/Type$TypeMnemonic;


# direct methods
.method static constructor <clinit>()V
    .registers 4

    .line 220
    new-instance v0, Lorg/xbill/DNS/Type$TypeMnemonic;

    invoke-direct {v0}, Lorg/xbill/DNS/Type$TypeMnemonic;-><init>()V

    sput-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    .line 223
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "A"

    new-instance v2, Lorg/xbill/DNS/ARecord;

    invoke-direct {v2}, Lorg/xbill/DNS/ARecord;-><init>()V

    const/4 v3, 0x1

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 224
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NS"

    new-instance v2, Lorg/xbill/DNS/NSRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NSRecord;-><init>()V

    const/4 v3, 0x2

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 225
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MD"

    new-instance v2, Lorg/xbill/DNS/MDRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MDRecord;-><init>()V

    const/4 v3, 0x3

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 226
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MF"

    new-instance v2, Lorg/xbill/DNS/MFRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MFRecord;-><init>()V

    const/4 v3, 0x4

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 227
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "CNAME"

    new-instance v2, Lorg/xbill/DNS/CNAMERecord;

    invoke-direct {v2}, Lorg/xbill/DNS/CNAMERecord;-><init>()V

    const/4 v3, 0x5

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 228
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "SOA"

    new-instance v2, Lorg/xbill/DNS/SOARecord;

    invoke-direct {v2}, Lorg/xbill/DNS/SOARecord;-><init>()V

    const/4 v3, 0x6

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 229
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MB"

    new-instance v2, Lorg/xbill/DNS/MBRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MBRecord;-><init>()V

    const/4 v3, 0x7

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 230
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MG"

    new-instance v2, Lorg/xbill/DNS/MGRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MGRecord;-><init>()V

    const/16 v3, 0x8

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 231
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MR"

    new-instance v2, Lorg/xbill/DNS/MRRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MRRecord;-><init>()V

    const/16 v3, 0x9

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 232
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NULL"

    new-instance v2, Lorg/xbill/DNS/NULLRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NULLRecord;-><init>()V

    const/16 v3, 0xa

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 233
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "WKS"

    new-instance v2, Lorg/xbill/DNS/WKSRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/WKSRecord;-><init>()V

    const/16 v3, 0xb

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 234
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "PTR"

    new-instance v2, Lorg/xbill/DNS/PTRRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/PTRRecord;-><init>()V

    const/16 v3, 0xc

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 235
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "HINFO"

    new-instance v2, Lorg/xbill/DNS/HINFORecord;

    invoke-direct {v2}, Lorg/xbill/DNS/HINFORecord;-><init>()V

    const/16 v3, 0xd

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 236
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MINFO"

    new-instance v2, Lorg/xbill/DNS/MINFORecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MINFORecord;-><init>()V

    const/16 v3, 0xe

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 237
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MX"

    new-instance v2, Lorg/xbill/DNS/MXRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/MXRecord;-><init>()V

    const/16 v3, 0xf

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 238
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "TXT"

    new-instance v2, Lorg/xbill/DNS/TXTRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/TXTRecord;-><init>()V

    const/16 v3, 0x10

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 239
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "RP"

    new-instance v2, Lorg/xbill/DNS/RPRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/RPRecord;-><init>()V

    const/16 v3, 0x11

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 240
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "AFSDB"

    new-instance v2, Lorg/xbill/DNS/AFSDBRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/AFSDBRecord;-><init>()V

    const/16 v3, 0x12

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 241
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "X25"

    new-instance v2, Lorg/xbill/DNS/X25Record;

    invoke-direct {v2}, Lorg/xbill/DNS/X25Record;-><init>()V

    const/16 v3, 0x13

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 242
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "ISDN"

    new-instance v2, Lorg/xbill/DNS/ISDNRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/ISDNRecord;-><init>()V

    const/16 v3, 0x14

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 243
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "RT"

    new-instance v2, Lorg/xbill/DNS/RTRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/RTRecord;-><init>()V

    const/16 v3, 0x15

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 244
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NSAP"

    new-instance v2, Lorg/xbill/DNS/NSAPRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NSAPRecord;-><init>()V

    const/16 v3, 0x16

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 245
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NSAP-PTR"

    new-instance v2, Lorg/xbill/DNS/NSAP_PTRRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NSAP_PTRRecord;-><init>()V

    const/16 v3, 0x17

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 246
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "SIG"

    new-instance v2, Lorg/xbill/DNS/SIGRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/SIGRecord;-><init>()V

    const/16 v3, 0x18

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 247
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "KEY"

    new-instance v2, Lorg/xbill/DNS/KEYRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/KEYRecord;-><init>()V

    const/16 v3, 0x19

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 248
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "PX"

    new-instance v2, Lorg/xbill/DNS/PXRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/PXRecord;-><init>()V

    const/16 v3, 0x1a

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 249
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "GPOS"

    new-instance v2, Lorg/xbill/DNS/GPOSRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/GPOSRecord;-><init>()V

    const/16 v3, 0x1b

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 250
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "AAAA"

    new-instance v2, Lorg/xbill/DNS/AAAARecord;

    invoke-direct {v2}, Lorg/xbill/DNS/AAAARecord;-><init>()V

    const/16 v3, 0x1c

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 251
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "LOC"

    new-instance v2, Lorg/xbill/DNS/LOCRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/LOCRecord;-><init>()V

    const/16 v3, 0x1d

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 252
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NXT"

    new-instance v2, Lorg/xbill/DNS/NXTRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NXTRecord;-><init>()V

    const/16 v3, 0x1e

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 253
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "EID"

    const/16 v2, 0x1f

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 254
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NIMLOC"

    const/16 v2, 0x20

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 255
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "SRV"

    new-instance v2, Lorg/xbill/DNS/SRVRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/SRVRecord;-><init>()V

    const/16 v3, 0x21

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 256
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "ATMA"

    const/16 v2, 0x22

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 257
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NAPTR"

    new-instance v2, Lorg/xbill/DNS/NAPTRRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NAPTRRecord;-><init>()V

    const/16 v3, 0x23

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 258
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "KX"

    new-instance v2, Lorg/xbill/DNS/KXRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/KXRecord;-><init>()V

    const/16 v3, 0x24

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 259
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "CERT"

    new-instance v2, Lorg/xbill/DNS/CERTRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/CERTRecord;-><init>()V

    const/16 v3, 0x25

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 260
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "A6"

    new-instance v2, Lorg/xbill/DNS/A6Record;

    invoke-direct {v2}, Lorg/xbill/DNS/A6Record;-><init>()V

    const/16 v3, 0x26

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 261
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "DNAME"

    new-instance v2, Lorg/xbill/DNS/DNAMERecord;

    invoke-direct {v2}, Lorg/xbill/DNS/DNAMERecord;-><init>()V

    const/16 v3, 0x27

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 262
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "OPT"

    new-instance v2, Lorg/xbill/DNS/OPTRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/OPTRecord;-><init>()V

    const/16 v3, 0x29

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 263
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "APL"

    new-instance v2, Lorg/xbill/DNS/APLRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/APLRecord;-><init>()V

    const/16 v3, 0x2a

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 264
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "DS"

    new-instance v2, Lorg/xbill/DNS/DSRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/DSRecord;-><init>()V

    const/16 v3, 0x2b

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 265
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "SSHFP"

    new-instance v2, Lorg/xbill/DNS/SSHFPRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/SSHFPRecord;-><init>()V

    const/16 v3, 0x2c

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 266
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "IPSECKEY"

    new-instance v2, Lorg/xbill/DNS/IPSECKEYRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/IPSECKEYRecord;-><init>()V

    const/16 v3, 0x2d

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 267
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "RRSIG"

    new-instance v2, Lorg/xbill/DNS/RRSIGRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/RRSIGRecord;-><init>()V

    const/16 v3, 0x2e

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 268
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NSEC"

    new-instance v2, Lorg/xbill/DNS/NSECRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NSECRecord;-><init>()V

    const/16 v3, 0x2f

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 269
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "DNSKEY"

    new-instance v2, Lorg/xbill/DNS/DNSKEYRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/DNSKEYRecord;-><init>()V

    const/16 v3, 0x30

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 270
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "DHCID"

    new-instance v2, Lorg/xbill/DNS/DHCIDRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/DHCIDRecord;-><init>()V

    const/16 v3, 0x31

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 271
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NSEC3"

    new-instance v2, Lorg/xbill/DNS/NSEC3Record;

    invoke-direct {v2}, Lorg/xbill/DNS/NSEC3Record;-><init>()V

    const/16 v3, 0x32

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 272
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "NSEC3PARAM"

    new-instance v2, Lorg/xbill/DNS/NSEC3PARAMRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/NSEC3PARAMRecord;-><init>()V

    const/16 v3, 0x33

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 273
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "SPF"

    new-instance v2, Lorg/xbill/DNS/SPFRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/SPFRecord;-><init>()V

    const/16 v3, 0x63

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 274
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "TKEY"

    new-instance v2, Lorg/xbill/DNS/TKEYRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/TKEYRecord;-><init>()V

    const/16 v3, 0xf9

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 275
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "TSIG"

    new-instance v2, Lorg/xbill/DNS/TSIGRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/TSIGRecord;-><init>()V

    const/16 v3, 0xfa

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 276
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "IXFR"

    const/16 v2, 0xfb

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 277
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "AXFR"

    const/16 v2, 0xfc

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 278
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MAILB"

    const/16 v2, 0xfd

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 279
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "MAILA"

    const/16 v2, 0xfe

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 280
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "ANY"

    const/16 v2, 0xff

    invoke-virtual {v0, v2, v1}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;)V

    .line 281
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    const-string v1, "DLV"

    new-instance v2, Lorg/xbill/DNS/DLVRecord;

    invoke-direct {v2}, Lorg/xbill/DNS/DLVRecord;-><init>()V

    const v3, 0x8001

    invoke-virtual {v0, v3, v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->add(ILjava/lang/String;Lorg/xbill/DNS/Record;)V

    .line 282
    return-void
.end method

.method private constructor <init>()V
    .registers 1

    .line 285
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 286
    return-void
.end method

.method public static check(I)V
    .registers 2
    .param p0, "val"    # I

    .line 294
    if-ltz p0, :cond_8

    const v0, 0xffff

    if-gt p0, v0, :cond_8

    .line 296
    return-void

    .line 295
    :cond_8
    new-instance v0, Lorg/xbill/DNS/InvalidTypeException;

    invoke-direct {v0, p0}, Lorg/xbill/DNS/InvalidTypeException;-><init>(I)V

    throw v0
.end method

.method static getProto(I)Lorg/xbill/DNS/Record;
    .registers 2
    .param p0, "val"    # I

    .line 335
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    invoke-virtual {v0, p0}, Lorg/xbill/DNS/Type$TypeMnemonic;->getProto(I)Lorg/xbill/DNS/Record;

    move-result-object v0

    return-object v0
.end method

.method public static isRR(I)Z
    .registers 2
    .param p0, "type"    # I

    .line 341
    const/16 v0, 0x29

    if-eq p0, v0, :cond_9

    packed-switch p0, :pswitch_data_c

    .line 352
    const/4 v0, 0x1

    return v0

    .line 350
    :cond_9
    :pswitch_9
    const/4 v0, 0x0

    return v0

    nop

    :pswitch_data_c
    .packed-switch 0xf9
        :pswitch_9
        :pswitch_9
        :pswitch_9
        :pswitch_9
        :pswitch_9
        :pswitch_9
        :pswitch_9
    .end packed-switch
.end method

.method public static string(I)Ljava/lang/String;
    .registers 2
    .param p0, "val"    # I

    .line 306
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    invoke-virtual {v0, p0}, Lorg/xbill/DNS/Type$TypeMnemonic;->getText(I)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public static value(Ljava/lang/String;)I
    .registers 2
    .param p0, "s"    # Ljava/lang/String;

    .line 330
    const/4 v0, 0x0

    invoke-static {p0, v0}, Lorg/xbill/DNS/Type;->value(Ljava/lang/String;Z)I

    move-result v0

    return v0
.end method

.method public static value(Ljava/lang/String;Z)I
    .registers 6
    .param p0, "s"    # Ljava/lang/String;
    .param p1, "numberok"    # Z

    .line 317
    sget-object v0, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    invoke-virtual {v0, p0}, Lorg/xbill/DNS/Type$TypeMnemonic;->getValue(Ljava/lang/String;)I

    move-result v0

    .line 318
    .local v0, "val":I
    const/4 v1, -0x1

    if-ne v0, v1, :cond_22

    if-eqz p1, :cond_22

    .line 319
    sget-object v1, Lorg/xbill/DNS/Type;->types:Lorg/xbill/DNS/Type$TypeMnemonic;

    new-instance v2, Ljava/lang/StringBuffer;

    invoke-direct {v2}, Ljava/lang/StringBuffer;-><init>()V

    const-string v3, "TYPE"

    invoke-virtual {v2, v3}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v2, p0}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v2}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-virtual {v1, v2}, Lorg/xbill/DNS/Type$TypeMnemonic;->getValue(Ljava/lang/String;)I

    move-result v0

    .line 321
    :cond_22
    return v0
.end method
