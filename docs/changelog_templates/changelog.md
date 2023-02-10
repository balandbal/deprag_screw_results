{% for version in [changelog.versions_list[0]] -%}
{%- if version.tag -%}
## [{{ version.tag }}]({{ version.compare_url }})<a name="{{ version.tag }}"></a>
{%- elif version.planned_tag -%}
## Upcoming {{ version.planned_tag }}<a name="{{ version.planned_tag }}"></a>
{%- else -%}
## Unreleased ([compare]({{ version.compare_url }})) <a name="Unrealeased"></a>
{%- endif -%}
{% if version.date %} ({{ version.date }})
{% endif -%}
{% for type, section in version.sections_dict|dictsort -%}
{%- if type %}
### {{ section.type or "Misc" }}
{% for commit in section.commits|sort(attribute='subject') %}
- {% if commit.style.scope %}**{{ commit.style.scope }}:** {% endif %}{{ commit.style.subject }} ([{{ commit.hash|truncate(7, True, '') }}]({{ commit.url }}))
{%- if commit.text_refs.issues_not_in_subject %}, related to {% for issue in commit.text_refs.issues_not_in_subject -%}
[{{ issue.ref }}]({{ issue.url }}){% if not loop.last %}, {% endif %}
{%- endfor -%}
{%- endif -%}
{% endfor %}
{% endif -%}
{%- endfor -%}
{% endfor -%}
