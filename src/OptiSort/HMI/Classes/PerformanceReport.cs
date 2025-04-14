using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Text.Json;
using System.Threading.Tasks;

namespace OptiSort.Classes
{
    internal class PerformanceReport
    {

        private DateTime _startTime;
        private string _cameraId;
        private int _initTimeMs;
        private int _workpieceLoadingCount = 0;


        [JsonExtensionData]
        public Dictionary<string, JsonElement> CombinedMetrics { get; set; } = new Dictionary<string, JsonElement>();

        public PerformanceReport(string cameraId, int initTimeMs)
        {
            _startTime = DateTime.UtcNow;
            _cameraId = cameraId;
            _initTimeMs = initTimeMs;

            // Add C# metrics right away
            CombinedMetrics["camera_id"] = JsonDocument.Parse($"\"{_cameraId}\"").RootElement;
            CombinedMetrics["init_time_ms"] = JsonDocument.Parse($"{_initTimeMs}").RootElement;
            CombinedMetrics["start_time"] = JsonDocument.Parse($"\"{_startTime:O}\"").RootElement;
        }


        public void UpdateWorkpieceLoadingCount()
        {
            _workpieceLoadingCount++;
        }


        public void MergePythonMetrics(JsonElement pythonMetrics)
        {
            if (pythonMetrics.ValueKind != JsonValueKind.Object)
                throw new ArgumentException("Expected a JSON object for Python metrics");

            foreach (var prop in pythonMetrics.EnumerateObject())
            {
                CombinedMetrics[prop.Name] = prop.Value;
            }
        }

        public string ToJsonLine()
        {
            return JsonSerializer.Serialize(CombinedMetrics);
        }

        public void ExportToJsonl(string path)
        {
            File.AppendAllText(path, ToJsonLine() + Environment.NewLine);
        }



    }
}
