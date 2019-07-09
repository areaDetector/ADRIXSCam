using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PVGenerator
{
    struct PV
    {
        public string name;
        public string description;
        public bool documentOnly;
        public bool perCCD;
        public string defaultValue;
        public string minimumValue;
        public string maximumValue;
        public string scale;
        public bool IsReadOnly;
        public bool IsPersistent;
        public string dataType;
        public string units;
        public string internalIndex;
        public List<string> enumStrings;

        // Positions of values within line (after include boolean is stripped off)
        const int namePos = 0;
        const int descriptionPos = namePos + 1;
        const int documentOnlyPos = descriptionPos + 1;
        const int perCCDPos = documentOnlyPos + 1;
        const int defaultValuePos = perCCDPos + 1;
        const int minimumValuePos = defaultValuePos + 1;
        const int maximumValuePos = minimumValuePos + 1;
        const int scalePos = maximumValuePos + 1;
        const int IsReadOnlyPos = scalePos + 1;
        const int IsPersistentPos = IsReadOnlyPos + 1;
        const int dataTypePos = IsPersistentPos + 1;
        const int unitsPos = dataTypePos + 1;
        const int internalIndexPos = unitsPos + 1;
        const int enumStringsPos = internalIndexPos + 1;

        public static PV FromStrings(string[] line)
        {
            PV pv = new PV()
            {
                name = line[namePos],
                description = line[descriptionPos],
                documentOnly = String.Equals(line[documentOnlyPos], "yes", StringComparison.OrdinalIgnoreCase),
                perCCD = String.Equals(line[perCCDPos], "yes", StringComparison.OrdinalIgnoreCase),
                defaultValue = line[defaultValuePos],
                minimumValue = line[minimumValuePos],
                maximumValue = line[maximumValuePos],
                scale = line[scalePos],
                IsReadOnly = !String.Equals(line[IsReadOnlyPos], "r/w", StringComparison.OrdinalIgnoreCase),
                IsPersistent = (!String.Equals(line[IsPersistentPos], "no", StringComparison.OrdinalIgnoreCase)),
                dataType = line[dataTypePos],
                units = line[unitsPos],
                internalIndex = line[internalIndexPos],
                enumStrings = null
            };

            if (pv.internalIndex.Length == 0)
                pv.internalIndex = "0";

            if (String.Equals(pv.dataType, "enum", StringComparison.OrdinalIgnoreCase))
            {
                pv.enumStrings = new List<string>();

                int i = enumStringsPos;
                while (line[i] != string.Empty)
                    pv.enumStrings.Add(line[i++]);

                pv.minimumValue = "0";
                pv.maximumValue = (i - enumStringsPos - 1).ToString();
            }

            return pv;
        }

        public bool IsValid()
        {
            return name.Length > 0;
        }
    }

    class Program
    {
        // The maximum number of CCDs that can be fitted
        // (Controls the count for MultiParameters)
        const int CCDCountMax = 3;

        // Names of the entries in the "menu" for enum-type PVs
        static string[] PVStringNames = { "ZRST", "ONST", "TWST", "THST", "FRST", "FVST", "SXST", "SVST", "EIST", "NIST" };
        // Names of the corresponding values
        static string[] PVValueNames = { "ZRVL", "ONVL", "TWVL", "THVL", "FRVL", "FVVL", "SXVL", "SVVL", "EIVL", "NIVL" };

        static Dictionary<string, string> dataTypeToRecordType = new Dictionary<string, string> 
        {
            {"int", "asynInt32"},
            {"float", "asynFloat64"}
        };

        static void Main(string[] args)
        {
            List<PV> pvs = new List<PV>();

            using (var reader = new StreamReader(args[0]))
            {
                string[] headings = SplitLine(reader.ReadLine());
                Debug.Assert(String.Equals(headings[1], "PV Name", StringComparison.OrdinalIgnoreCase));

                while (reader.Peek() >= 0)
                {
                    string[] strings = SplitLine(reader.ReadLine());
                    // Should it be included?
                    if (String.Equals(strings[0], "yes", StringComparison.OrdinalIgnoreCase))
                    {
                        PV pv = PV.FromStrings(strings.Skip(1).ToArray());
                        if (pv.IsValid())
                            pvs.Add(pv);
                    }
                }
            }

            Debug.WriteLine("Read csv file...");

            string subsystemName = args[1];

            List<PV> pvsImplement = pvs.Where(pv => !pv.documentOnly).ToList();

            using (var writer = new StreamWriter(Path.GetDirectoryName(args[0]) + "\\PVDeclarations.h"))
            {
                string indent = "      ";

                writer.WriteLine("// File generated by PVGenerator");
                writer.WriteLine("// Intended to be #included into header file for an EPICS areaDetector IOC class (derived from ADDriver)");
                writer.WriteLine(indent + string.Format("const static int _ccdCountMax = {0};", CCDCountMax));

                int parameterCount = 0;
                foreach (var pv in pvsImplement)
                {
                    string param = indent;
                    parameterCount += 1;

                    if (pv.IsReadOnly)
                        param += "ReadOnlyParameter<";
                    else
                    {
                        if (pv.perCCD)
                        {
                            param += "MultiParameter<_ccdCountMax, ";
                            parameterCount += CCDCountMax - 1; // Already added 1
                        }
                        else
                            param += "Parameter<";
                    }

                    switch (pv.dataType)
                    {
                        case "int":
                        case "enum":
                            param += "epicsInt32";
                            break;

                        case "float":
                            param += "epicsFloat64";
                            break;

                        default:
                            Debug.Assert(false);
                            break;
                    }

                    param += string.Format("> _param{0}{{\"{0}\", {1}", pv.name, pv.internalIndex);

                    if (!pv.IsReadOnly)
                    {
                        param += string.Format(", {0}, {1}", pv.minimumValue, pv.maximumValue);
                        param += string.Format(", {0}", pv.defaultValue);
                    }
                    if (pv.scale.Length > 0)
                        param += string.Format(", {0}", pv.scale);

                    param += "};";

                    writer.WriteLine(param);
                }

                writer.WriteLine();
                writer.WriteLine(indent + string.Format("const static int _parameterContainerCount = {0};", pvsImplement.Count));
                writer.WriteLine(indent + string.Format("const static int _parameterCount = {0};", parameterCount));

                writer.WriteLine();
                writer.WriteLine(indent + "std::vector<ParameterBase*> _allParams;");
            }

            using (var writer = new StreamWriter(Path.GetDirectoryName(args[0]) + "\\PVDefinitions.cpp"))
            {
                writer.WriteLine("// File generated by PVGenerator");
                writer.WriteLine("// Intended to be #included into constructor code for an EPICS areaDetector IOC class (derived from ADDriver)");
                string indent = "    ";

                // Code to initialize the contents of the _allParams collection

                writer.WriteLine(indent + "_allParams = {");

                foreach (var pv in pvsImplement)
                    writer.WriteLine(indent + indent + "&_param" + pv.name + ","); 

                // (comma on final entry seems to be ignored!)

	            writer.WriteLine(indent + "};");
            }

            // Database definitions file
            using (var writer = new StreamWriter(Path.GetDirectoryName(args[0]) + "\\" + subsystemName + "PVDefinitions.template"))
            {
                writer.WriteLine("# Template file generated by PVGenerator");
                writer.WriteLine("# Intended to be included into manually-produced template file");
                writer.WriteLine();

                foreach (var pv in pvsImplement)
                    WritePVTemplate(writer, pv);
            }

            // Autosave definitions
            // Need to use specific subsystem name here, because need to combine them independently
            using (var writer = new StreamWriter(Path.Combine(Path.GetDirectoryName(args[0]) , subsystemName + ".req")))
            {
                writer.WriteLine("# Autosave definitions file generated by PVGenerator");
                writer.WriteLine("# Intended to be included into auto_settings.req");
                writer.WriteLine();

                // Here we use all the pvs, irrespective of whether we implement them
                foreach (var pv in pvs)
                {
                    if (!pv.IsReadOnly && pv.IsPersistent)
                    {
                        if (pv.perCCD)
                        {
                            for (int ccd = 0; ccd < CCDCountMax; ++ccd)
                            {
                                // CCDs have suffices _1, _2, _3
                                string fullName = pv.name + "_" + (ccd + 1).ToString();
                                writer.WriteLine("$(P)$(R)" + fullName);
                            }
                        }
                        else
                            writer.WriteLine("$(P)$(R)" + pv.name);
                    }
                }
            }

            using (var reader = new StreamReader(Path.GetDirectoryName(args[0]) + "\\DocTemplate.html"))
            {
                using (var writer = new StreamWriter(Path.GetDirectoryName(args[0]) + "\\Doc.html"))
                {
                    while (reader.Peek() >= 0)
                    {
                        string line = reader.ReadLine();
                        if (line.Trim() == "<!-- Insert record definitions here -->")
                            break;
                        writer.WriteLine(line);
                    }

                    string indent = "   ";
                    string indent3 = indent + indent + indent;

                    foreach (var pv in pvs)
                    {
                        writer.WriteLine(indent3 + "<tr>");

                        WriteTableEntry(writer, indent3 + indent, indent, pv.IsReadOnly ? "r/o" : "r/w");
                        WriteTableEntry(writer, indent3 + indent, indent, pv.description);
                        string name = pv.name;
                        if (pv.perCCD)
                            name += "_$(CCD)";
                        WriteTableEntry(writer, indent3 + indent, indent, name);
                        string EPICSRecordName = "$(P)$(R)" + name;
                        if (!pv.IsReadOnly)
                            EPICSRecordName += "<br />" + EPICSRecordName + "_RBV";
                        WriteTableEntry(writer, indent3 + indent, indent, EPICSRecordName);

                        string EPICSRecordType = string.Empty;
                        switch (pv.dataType.ToLowerInvariant())
                        {
                            case "enum":
                                EPICSRecordType = "mbbi";
                                if (!pv.IsReadOnly)
                                    EPICSRecordType += "<br />" + "mbbo";
                                break;

                            case "float":
                                EPICSRecordType = "ai";
                                if (!pv.IsReadOnly)
                                    EPICSRecordType += "<br />" + "ao";
                                break;

                            case "int":
                                EPICSRecordType = "longin";
                                if (!pv.IsReadOnly)
                                    EPICSRecordType += "<br />" + "longout";
                                break;

                            case "string":
                                EPICSRecordType = "stringin";
                                if (!pv.IsReadOnly)
                                    EPICSRecordType += "<br />" + "stringout";
                                break;

                            default:
                                Debug.Assert(false);
                                break;
                        }

                        WriteTableEntry(writer, indent3 + indent, indent, EPICSRecordType);
                        writer.WriteLine(indent3 + "</tr>");
                    }

                    while (reader.Peek() >= 0)
                    {
                        writer.WriteLine(reader.ReadLine());
                    }

                }
            }
        }

        static private void WriteTableEntry(StreamWriter writer, string indentBase, string indent, string entry)
        {
            writer.WriteLine(indentBase + "<td>");
            writer.WriteLine(indentBase + indent + entry);
            writer.WriteLine(indentBase + "</td>");
        }

        static private void WritePVTemplate(StreamWriter writer, PV pv)
        {
            if (pv.perCCD)
            {
                for (int ccd = 0; ccd < CCDCountMax; ++ccd)
                {
                    // CCDs have suffices _1, _2, _3
                    string fullName = pv.name + "_" + (ccd + 1).ToString();
                    WritePVTemplate(writer, pv, fullName);
                }
            }
            else
                WritePVTemplate(writer, pv, pv.name);
        }

        static private void WritePVTemplate(StreamWriter writer, PV pv, string name)
        {
            if (!pv.IsReadOnly)
                WritePVTemplate(writer, pv, name, false);

            WritePVTemplate(writer, pv, name, true);
        }

        static private void WritePVTemplate(StreamWriter writer, PV pv, string name, bool readbackValue)
        {
            string indent = "   ";

            string recordName = name;
            if (readbackValue)
                recordName += "_RBV";

            string recordType = string.Empty;
            switch (pv.dataType.ToLowerInvariant())
            {
                case "enum":
                    recordType = (readbackValue ? "mbbi" : "mbbo");
                    break;

                case "float":
                    recordType = (readbackValue ? "ai" : "ao");
                    break;

                case "int":
                    recordType = (readbackValue ? "longin" : "longout");
                    break;

                default:
                    Debug.Assert(false);
                    break;
            }

            writer.WriteLine(string.Format("record({0}, \"$(P)$(R){1}\")", recordType, recordName));
            writer.WriteLine("{");

            if (!readbackValue)
                WriteTemplateLine(writer, indent, "field", "PINI", "YES");

            switch (pv.dataType.ToLowerInvariant())
            {
                case "enum":
                    {
                        WriteTemplateLine(writer, indent, "field", "DTYP", "asynInt32");
                        WriteTemplateLine(writer, indent, "field", (readbackValue ? "INP" : "OUT"), "@asyn($(PORT),$(ADDR),$(TIMEOUT))" + name);
                        for (int i = 0; i < pv.enumStrings.Count; ++i)
                        {
                            WriteTemplateLine(writer, indent, "field", PVStringNames[i], pv.enumStrings[i]);
                            WriteTemplateLine(writer, indent, "field", PVValueNames[i], i.ToString());
                        }
                        break;
                    }

                case "float":
                        WriteTemplateLine(writer, indent, "field", "PREC", "2");
                        goto case "int";

                case "int":
                    {
                        WriteTemplateLine(writer, indent, "field", "DTYP", dataTypeToRecordType[pv.dataType]);
                        WriteTemplateLine(writer, indent, "field", (readbackValue ? "INP" : "OUT"), "@asyn($(PORT),$(ADDR),$(TIMEOUT))" + name);

                        WriteTemplateLineOption(writer, indent, "HOPR", pv.maximumValue);
                        WriteTemplateLineOption(writer, indent, "LOPR", pv.minimumValue);
                        break;
                    }
            }

            WriteTemplateLineOption(writer, indent, "VAL", pv.defaultValue);

            WriteTemplateLineOption(writer, indent, "EGU", pv.units);

            if (readbackValue)
                WriteTemplateLine(writer, indent, "field", "SCAN", "I/O Intr");


            if (!readbackValue && pv.IsPersistent)
                WriteTemplateLine(writer, indent, "info", "autosavefields", "VAL");

            writer.WriteLine("}");
        }

        static private void WriteTemplateLineOption(StreamWriter writer, string indent, string fieldName, string pvValue)
        {
            if (pvValue != string.Empty)
                WriteTemplateLine(writer, indent, "field", fieldName, pvValue);
        }

        static private void WriteTemplateLine(StreamWriter writer, string indent, string a, string b, string c)
        {
            writer.WriteLine(indent + a + "(" + b + ", " + QuotedString(c) + ")");
        }

        static private string[] SplitLine(string line)
        {
            return line.Split(new char[] { ',' }).Select(s => s.Trim()).ToArray();
        }

        static string QuotedString(string s)
        {
            return "\"" + s + "\"";
        }
    }
}


