.class public interface abstract Ldiagnostic_msgs/AddDiagnosticsRequest;
.super Ljava/lang/Object;
.source "AddDiagnosticsRequest.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final _DEFINITION:Ljava/lang/String; = "# This service is used as part of the process for loading analyzers at runtime,\n# and should be used by a loader script or program, not as a standalone service.\n# Information about dynamic addition of analyzers can be found at\n# http://wiki.ros.org/diagnostics/Tutorials/Adding%20Analyzers%20at%20Runtime\n\n# The load_namespace parameter defines the namespace where parameters for the\n# initialization of analyzers in the diagnostic aggregator have been loaded. The\n# value should be a global name (i.e. /my/name/space), not a relative\n# (my/name/space) or private (~my/name/space) name. Analyzers will not be added\n# if a non-global name is used. The call will also fail if the namespace\n# contains parameters that follow a namespace structure that does not conform to\n# that expected by the analyzer definitions. See\n# http://wiki.ros.org/diagnostics/Tutorials/Configuring%20Diagnostic%20Aggregators\n# and http://wiki.ros.org/diagnostics/Tutorials/Using%20the%20GenericAnalyzer\n# for examples of the structure of yaml files which are expected to have been\n# loaded into the namespace.\nstring load_namespace\n"

.field public static final _TYPE:Ljava/lang/String; = "diagnostic_msgs/AddDiagnosticsRequest"


# virtual methods
.method public abstract getLoadNamespace()Ljava/lang/String;
.end method

.method public abstract setLoadNamespace(Ljava/lang/String;)V
.end method
